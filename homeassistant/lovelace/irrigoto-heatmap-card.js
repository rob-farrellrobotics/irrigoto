/*
 * irrigoto-heatmap-card.js — Lovelace custom card for Home Assistant
 *
 * Renders the per-zone watering depth heatmap from an irrigoto device,
 * matching the on-device "Actual" mode rendering in zone_setup.html.
 *
 * Install:
 *   1. Copy this file to <ha-config>/www/community/irrigoto/ (or
 *      anywhere under www/). Example final path:
 *        config/www/community/irrigoto/irrigoto-heatmap-card.js
 *   2. Add it as a Lovelace resource (Settings → Dashboards → Resources):
 *        URL:  /local/community/irrigoto/irrigoto-heatmap-card.js
 *        Type: JavaScript Module
 *   3. Add a card to any dashboard. Pick ONE of:
 *
 *      Preferred -- HA-managed (no IP hardcoded, follows DHCP):
 *        type: custom:irrigoto-heatmap-card
 *        device: irrigoto-xxxxxx     # ESPHome device slug or HA device name
 *        zone_id: 0
 *
 *      Explicit URL (override -- use if auto-discovery fails):
 *        type: custom:irrigoto-heatmap-card
 *        device_url: http://192.168.1.x
 *        zone_id: 0
 *
 *      Common options for either form:
 *        title: "Zone heatmap"        # optional
 *        refresh_interval: 0          # optional, seconds; 0 disables
 *
 * Auto-discovery looks up the device in HA's device registry and uses
 * its `configuration_url` (which the ESPHome integration keeps current
 * with the device's live IP). This means DHCP renewals don't break
 * anything -- HA tracks the IP and the card picks it up at render.
 *
 * Fetches three endpoints from the device on render:
 *   GET /api/all                    -- zone polygon (perimeter vertices)
 *   GET /zone/last_water?id=N       -- per-ring summary
 *   GET /zone/water_csv?id=N        -- per-(ring,sector) aggregate CSV
 *
 * No HA REST sensors required. The card fetches directly from the
 * device, same way the on-device web UI does. Requires the HA browser
 * to be able to reach the device (same LAN, or a working VPN).
 * Remote access via Nabu Casa does NOT work for this card unless you've
 * separately exposed the device's HTTP endpoints to the cloud.
 */

class IrrigotoHeatmapCard extends HTMLElement {
  static getStubConfig() {
    return { device: "irrigoto-xxxxxx", zone_id: 0 };
  }

  setConfig(config) {
    if (!config || (!config.device && !config.device_url)) {
      throw new Error("either 'device' (HA device name) or 'device_url' is required");
    }
    if (config.zone_id === undefined || config.zone_id === null) {
      throw new Error("zone_id is required");
    }
    this._config = {
      device: config.device || null,
      device_url: config.device_url
        ? String(config.device_url).replace(/\/+$/, "")
        : null,
      // Per-device HA mirror sub-folder. The mirror shell_command writes
      // to /config/www/community/irrigoto/<slug>/csv/, so each device's
      // cached heatmap files stay separate. Defaults to the device slug
      // when not given (legacy flat layout if neither is set).
      mirror_slug: config.mirror_slug || config.device || null,
      zone_id: Number(config.zone_id),
      title: config.title || `Zone ${config.zone_id} heatmap`,
      refresh_interval: Number(config.refresh_interval || 0),
      max_throw_mm: Number(config.max_throw_mm || 8534),
      target_depth_mm: Number(config.target_depth_mm || 3.175),
    };
    this._resolvedUrl = null; // cache for the lookup result

    if (!this._initialized) this._buildDom();
    this._title.textContent = this._config.title;
    this._scheduleLoad();
    this._setupAutoRefresh();
  }

  set hass(hass) {
    const wasUnset = !this._hass;
    this._hass = hass;
    // First time hass arrives we need to retry the load -- the initial
    // setConfig fired before hass was set, so device-registry lookup
    // returned an "hass not available" error and was rendered as
    // "Loading…" perpetually.
    if (wasUnset && this._config && this._config.device && !this._config.device_url) {
      this._scheduleLoad();
      return;
    }
    // Always re-render the info row on hass update once we have data.
    // HA fires set hass on every state change in the system, so this
    // runs frequently -- but _renderInfo is just an innerHTML update
    // on a small div, cheap. The benefit: stats appear without manual
    // refresh whenever the per-zone sensors populate, including the
    // common "first card paint happened with hass not-yet-fully-loaded"
    // case where sensor states arrived a few hundred ms after _loadData
    // finished its initial render.
    if (this._lastRun && this._info) {
      this._renderInfo();
    }
  }

  getCardSize() {
    return 6;
  }

  connectedCallback() {
    this._setupAutoRefresh();
  }

  disconnectedCallback() {
    if (this._refreshTimer) {
      clearInterval(this._refreshTimer);
      this._refreshTimer = null;
    }
  }

  _setupAutoRefresh() {
    if (this._refreshTimer) {
      clearInterval(this._refreshTimer);
      this._refreshTimer = null;
    }
    const sec = this._config && this._config.refresh_interval;
    if (sec && sec > 0) {
      this._refreshTimer = setInterval(() => this._scheduleLoad(), sec * 1000);
    }
  }

  _buildDom() {
    this._initialized = true;
    this.innerHTML = "";
    const card = document.createElement("ha-card");
    const wrap = document.createElement("div");
    wrap.className = "irrigoto-wrap";
    // Layout: header on top, then a 2-column flex row -- canvas on the
    // left, vertical legend (color bar + ticks + caption) on the right.
    // Info row at the bottom. The legend mirrors the device's depth color
    // ramp: bottom = blue (0), middle = green (1x), top = red (2x+).
    // Layout: header on top, then canvas with the color-ramp legend
    // as an absolute-positioned overlay on the right edge of the
    // canvas (frees up all horizontal width for the heatmap itself).
    // The caption "depth vs target" is rotated to read vertically,
    // bottom→top, so it fits in the narrow legend column without
    // taking horizontal space. Info row at the bottom.
    wrap.innerHTML = `
      <div class="irrigoto-header">
        <span class="irrigoto-title"></span>
        <button class="irrigoto-refresh" title="Refresh">↻</button>
      </div>
      <div class="irrigoto-canvas-wrap">
        <canvas class="irrigoto-canvas"></canvas>
        <div class="irrigoto-status"></div>
        <div class="irrigoto-legend">
          <span class="irrigoto-legend-caption">depth vs target</span>
          <span class="irrigoto-legend-labels">
            <span>2x+</span>
            <span>1.5x</span>
            <span>1x</span>
            <span>0.5x</span>
            <span>0</span>
          </span>
          <span class="irrigoto-legend-bar"></span>
        </div>
      </div>
      <div class="irrigoto-info"></div>
    `;
    const style = document.createElement("style");
    style.textContent = `
      .irrigoto-wrap { padding: 6px; display: flex; flex-direction: column; gap: 4px; }
      .irrigoto-header { display: flex; align-items: center; justify-content: space-between; min-height: 0; }
      .irrigoto-title { font-size: 0.95rem; font-weight: 500; color: var(--primary-text-color); }
      .irrigoto-refresh {
        background: transparent; border: 1px solid var(--divider-color);
        border-radius: 4px; color: var(--secondary-text-color);
        font-size: 0.95rem; cursor: pointer; padding: 0 6px; line-height: 1.4;
      }
      .irrigoto-refresh:hover { color: var(--primary-color); border-color: var(--primary-color); }
      .irrigoto-canvas-wrap { position: relative; width: 100%; min-width: 0; }
      .irrigoto-canvas { width: 100%; aspect-ratio: 1 / 1; background: var(--card-background-color); display: block; }
      .irrigoto-status {
        position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
        color: var(--secondary-text-color); font-size: 0.95rem; text-align: center;
        pointer-events: none;
      }
      .irrigoto-status.error { color: var(--error-color, #f44336); }
      /* Cached-fallback badge -- shown when device fetch failed and
         we're rendering from localStorage. Small pill in top-LEFT
         corner so it doesn't collide with the right-edge legend. */
      .irrigoto-status.cached {
        inset: auto auto auto auto;
        top: 6px; left: 6px;
        align-items: flex-start; justify-content: flex-start;
        font-size: 0.7rem;
        background: rgba(255,140,0,0.18);
        color: rgba(255,170,40,0.95);
        border: 1px solid rgba(255,140,0,0.45);
        border-radius: 10px;
        padding: 2px 8px;
        white-space: nowrap;
      }
      /* Color-ramp legend overlay: lives inside the canvas-wrap, right
         edge, vertically centered. Slim profile (~16px total CSS width)
         so the centered radar only has to shrink ~16px diameter to
         fully clear it. */
      .irrigoto-legend {
        position: absolute;
        top: 10%; bottom: 10%; right: 2px;
        display: flex; flex-direction: row;
        align-items: stretch; gap: 1px;
        font-size: 0.5rem; color: var(--secondary-text-color);
        pointer-events: none;
      }
      .irrigoto-legend-caption {
        /* "depth vs target" rotated to read bottom→top so it stacks
           vertically alongside the color bar without claiming any
           horizontal space. sideways-lr is the spec-correct way to
           get bottom-up text flow. */
        writing-mode: sideways-lr;
        text-align: center;
        white-space: nowrap;
        letter-spacing: 0;
        opacity: 0.75;
        font-size: 0.5rem;
      }
      .irrigoto-legend-bar {
        width: 5px; border-radius: 2px;
        background: linear-gradient(to top,
          rgba(40,90,200,0.85)  0%,
          rgba(40,200,180,0.85) 30%,
          rgba(60,220,80,0.95)  50%,
          rgba(255,195,40,0.95) 75%,
          rgba(255,90,40,0.95)  100%);
        box-shadow: 0 0 0 1px rgba(0,0,0,0.35);
      }
      .irrigoto-legend-labels {
        display: flex; flex-direction: column; justify-content: space-between;
        font-size: 0.5rem; line-height: 1;
        text-shadow: 0 0 3px rgba(0,0,0,0.7);
      }
      .irrigoto-info {
        font-size: 0.72rem; color: var(--secondary-text-color);
        display: flex; flex-wrap: wrap; gap: 2px 10px;
        line-height: 1.4;
      }
      .irrigoto-info span { white-space: nowrap; }
      .irrigoto-info b { color: var(--primary-text-color); font-weight: 500; }
      .irrigoto-info span.dim { opacity: 0.55; }
      .irrigoto-info span.flag-warn b { color: var(--warning-color, #ff9800); }
    `;
    card.appendChild(style);
    card.appendChild(wrap);
    this.appendChild(card);

    this._title = wrap.querySelector(".irrigoto-title");
    this._canvas = wrap.querySelector(".irrigoto-canvas");
    this._status = wrap.querySelector(".irrigoto-status");
    this._info = wrap.querySelector(".irrigoto-info");
    wrap.querySelector(".irrigoto-refresh").addEventListener("click", () => {
      this._loadData(true);
    });
  }

  _scheduleLoad() {
    if (this._loadScheduled) return;
    this._loadScheduled = true;
    queueMicrotask(() => {
      this._loadScheduled = false;
      this._loadData(false);
    });
  }

  async _resolveDeviceUrl() {
    // Explicit device_url always wins (escape hatch when auto-discovery
    // can't reach the device, e.g. multi-LAN, custom mDNS issues, etc.)
    if (this._config.device_url) return this._config.device_url;

    if (this._resolvedUrl) return this._resolvedUrl;
    if (!this._hass) throw new Error("HA connection not ready yet");

    // Query HA's device registry. ESPHome populates each device's
    // configuration_url with the live device IP.
    const devices = await this._hass.callWS({
      type: "config/device_registry/list",
    });
    const want = String(this._config.device).toLowerCase();
    const wantNoSep = want.replace(/[-_]/g, "");

    // Match strategies, in order of strictness:
    //   1. Exact (case-insensitive) on name / name_by_user / identifier[1]
    //   2. Case-insensitive after stripping - and _ on those same fields
    //      (covers "Irrigoto-XXXXXX" vs "irrigoto_xxxxxx" vs "irrigoto-xxxxxx")
    //   3. Substring: device's name contains want, or want contains it
    //      (covers "Irrigoto" matching "irrigoto-xxxxxx")
    // Whichever hits first wins.
    const candidatesFor = (d) => {
      const out = [];
      if (d.name) out.push(String(d.name));
      if (d.name_by_user) out.push(String(d.name_by_user));
      if (d.identifiers) {
        for (const tuple of d.identifiers) {
          if (tuple && tuple.length >= 2 && tuple[1])
            out.push(String(tuple[1]));
        }
      }
      return out;
    };

    let match = null;
    let strategy = null;

    // Strategy 1: exact
    match = devices.find((d) =>
      candidatesFor(d).some((c) => c.toLowerCase() === want)
    );
    if (match) strategy = "exact";

    // Strategy 2: normalized (strip - and _)
    if (!match) {
      match = devices.find((d) =>
        candidatesFor(d).some(
          (c) => c.toLowerCase().replace(/[-_\s]/g, "") === wantNoSep
        )
      );
      if (match) strategy = "normalized";
    }

    // Strategy 3: substring (either direction)
    if (!match) {
      match = devices.find((d) =>
        candidatesFor(d).some((c) => {
          const cl = c.toLowerCase();
          return cl.includes(want) || want.includes(cl);
        })
      );
      if (match) strategy = "substring";
    }

    if (!match) {
      // Surface what HA actually has so the user can fix the config.
      // Filter to devices that have a configuration_url -- those are
      // the ones a heatmap card could conceivably point at. Limit to
      // 20 names to avoid flooding the console with HA core devices.
      const known = devices
        .filter((d) => d.configuration_url)
        .slice(0, 20)
        .map((d) => candidatesFor(d).join(" | "));
      console.log(
        "[irrigoto-heatmap-card] Available devices with configuration_url (first 20):\n%s",
        known.join("\n")
      );
      throw new Error(
        `Device '${this._config.device}' not found. ` +
          `Check console for available device names, or set device_url ` +
          `explicitly in the card config.`
      );
    }
    // Debug: dump the matched device record so we can see what HA
    // actually has. Many users assume configuration_url is set (since
    // HA itself talks to the device over native API and definitely
    // knows the IP) but in practice ESPHome integration only sets
    // configuration_url when web_server is exposed AND the
    // integration's been updated through a recent enough HA. Logging
    // the full record makes any field mismatch obvious.
    console.log(
      "[irrigoto-heatmap-card] matched device record: %o",
      match
    );

    // Prefer configuration_url if present (set by the ESPHome
    // integration's auto-adoption flow). If not present -- e.g.
    // device adopted via the ESPHome dashboard add-on -- fall back
    // to the matching config entry's data.host. ESPHome config
    // entries always carry the device IP in data.host since that's
    // how HA connects to the native API.
    let resolved = null;
    let resolvedVia = null;
    if (match.configuration_url) {
      resolved = match.configuration_url;
      resolvedVia = "configuration_url";
    } else if (match.config_entries && match.config_entries.length > 0) {
      // Multiple paths to try; HA's config entry data is partially
      // redacted from the frontend but each HA version exposes it
      // differently. Walk them all and log results so we know what's
      // actually accessible.
      try {
        const entries = await this._hass.callWS({
          type: "config_entries/get",
          domain: "esphome",
        });
        console.log(
          "[irrigoto-heatmap-card] config_entries/get returned %d esphome entries: %o",
          (entries || []).length,
          entries
        );
        const entry = (entries || []).find((e) =>
          match.config_entries.includes(e.entry_id)
        );
        if (entry) {
          console.log(
            "[irrigoto-heatmap-card] matched config entry: %o",
            entry
          );
          // Try entry.data.host directly first.
          let host = entry.data && entry.data.host;

          // Try the per-entry REST endpoint.
          if (!host) {
            try {
              const detail = await this._hass.callApi(
                "GET",
                `config/config_entries/entry/${entry.entry_id}`
              );
              console.log(
                "[irrigoto-heatmap-card] entry detail (REST): %o",
                detail
              );
              host = detail && detail.data && detail.data.host;
            } catch (e) {
              console.log(
                "[irrigoto-heatmap-card] entry REST detail failed: %s",
                e.message
              );
            }
          }

          // Try diagnostics endpoint -- some HA versions expose host here.
          if (!host) {
            try {
              const diag = await this._hass.callApi(
                "GET",
                `diagnostics/${entry.entry_id}`
              );
              console.log(
                "[irrigoto-heatmap-card] diagnostics: %o",
                diag
              );
              // Diagnostics structure varies; look for common patterns.
              if (diag && diag.data) {
                host = diag.data.host || (diag.data.entry && diag.data.entry.data && diag.data.entry.data.host);
              }
            } catch (e) {
              /* swallow */
            }
          }

          if (host) {
            resolved = `http://${host}`;
            resolvedVia = `config_entries/get (host=${host})`;
          }
        } else {
          console.log(
            "[irrigoto-heatmap-card] no esphome config entry matched device.config_entries=%o",
            match.config_entries
          );
        }
      } catch (e) {
        console.log(
          "[irrigoto-heatmap-card] config_entries lookup failed: %s",
          e.message
        );
      }
    }

    // Next-best fallback: look for an "IP Address" sensor that
    // belongs to this device. ESPHome exposes wifi_info.ip_address
    // as a regular text_sensor in HA -- entity state IS the IP.
    // Bypasses HA's config_entry.data redaction entirely. Requires
    // the device's ESPHome YAML to include:
    //   text_sensor:
    //     - platform: wifi_info
    //       ip_address:
    //         name: "..."
    // The matching entity has device_class "ip" or its entity_id
    // ends in _ip_address. Walk the entity registry filtered by
    // device_id and find one matching either criterion. Read the
    // entity's state from hass.states for the live IP.
    if (!resolved) {
      try {
        const entityRegistry = await this._hass.callWS({
          type: "config/entity_registry/list",
        });
        const myEntities = (entityRegistry || []).filter(
          (e) => e.device_id === match.id
        );
        // Look for an IP-shaped entity. Match in priority order:
        //   1. entity_id ending in '_ip_address'
        //   2. translation_key === 'ip_address'
        //   3. state matches IPv4 dotted form
        let ipEntity = myEntities.find((e) =>
          (e.entity_id || "").toLowerCase().endsWith("_ip_address")
        );
        if (!ipEntity) {
          ipEntity = myEntities.find(
            (e) => e.translation_key === "ip_address"
          );
        }
        if (!ipEntity) {
          ipEntity = myEntities.find((e) => {
            const st = this._hass.states[e.entity_id];
            return st && /^\d+\.\d+\.\d+\.\d+$/.test(st.state || "");
          });
        }
        if (ipEntity) {
          const st = this._hass.states[ipEntity.entity_id];
          if (st && /^\d+\.\d+\.\d+\.\d+$/.test(st.state || "")) {
            resolved = `http://${st.state}`;
            resolvedVia = `ip_address entity (${ipEntity.entity_id})`;
          } else {
            console.log(
              "[irrigoto-heatmap-card] IP entity %s has non-IP state: %o",
              ipEntity.entity_id,
              st && st.state
            );
          }
        }
      } catch (e) {
        console.log(
          "[irrigoto-heatmap-card] entity_registry lookup failed: %s",
          e.message
        );
      }
    }

    // Last fallback: construct the mDNS hostname from the device's
    // slug and try http://<slug>.local. This works when the user's
    // browser resolves .local addresses (most desktops; iOS/macOS;
    // many Android setups). Limits: HTTPS HA + HTTP .local URL is
    // mixed-content blocked in some browsers.
    if (!resolved) {
      const slug = String(
        match.name_by_user || match.name || this._config.device
      )
        .toLowerCase()
        .replace(/\s+/g, "-")
        .replace(/[^a-z0-9-]/g, "");
      if (slug) {
        resolved = `http://${slug}.local`;
        resolvedVia = `mDNS hostname (${slug}.local)`;
      }
    }

    if (!resolved) {
      throw new Error(
        `Cannot resolve URL for '${this._config.device}'. To fix:\n` +
          `  (recommended) In HA: Settings → Devices → "${
            match.name || match.name_by_user || "this device"
          }" → edit, set the Visit/Configuration URL to http://<your-device-ip>.\n` +
          `  (or) Set device_url: "http://<ip>" in the card YAML config to override.`
      );
    }

    console.log(
      "[irrigoto-heatmap-card] resolved '%s' -> '%s' (match: %s, via: %s, url: %s)",
      this._config.device,
      match.name || match.name_by_user || "?",
      strategy,
      resolvedVia,
      resolved
    );
    this._resolvedUrl = resolved.replace(/\/+$/, "");
    return this._resolvedUrl;
  }

  // Once we have a fetched/cached zone object, swap the card title
  // from the static config title ("Zone N heatmap") to a more
  // informative "Zone <1-based-id> <zone_name>". Falls back silently
  // if the zone name isn't present yet (very first load before any
  // data exists, or zone never had a name configured on the device).
  _patchTitle() {
    if (!this._title || !this._zone) return;
    const name = this._zone.name ? String(this._zone.name).trim() : "";
    if (!name) return;
    this._title.textContent = `Zone ${this._config.zone_id + 1} ${name}`;
  }

  _cacheKey() {
    // Keyed by device id (or url) + zone so multiple cards sharing
    // localStorage don't collide. Both forms are accepted -- whichever
    // the user configured.
    const dev = this._config.device || this._config.device_url || "unknown";
    return `irrigoto-heatmap-cache:${dev}:${this._config.zone_id}`;
  }

  _saveCache(zone, lastWater, csvText) {
    try {
      const payload = JSON.stringify({
        ts: Date.now(),
        zone,
        lastWater,
        csvText,
      });
      localStorage.setItem(this._cacheKey(), payload);
    } catch (e) {
      // Quota exceeded or storage disabled -- not fatal, just lose
      // the cache. Log once so it's visible if the user wonders why
      // offline persistence isn't working.
      console.log(
        "[irrigoto-heatmap-card] zone %d: cache save failed (%s)",
        this._config.zone_id,
        e.message,
      );
    }
  }

  _loadCache() {
    try {
      const raw = localStorage.getItem(this._cacheKey());
      if (!raw) return null;
      return JSON.parse(raw);
    } catch (e) {
      return null;
    }
  }

  _ageStr(tsMs) {
    const ageMin = Math.round((Date.now() - tsMs) / 60000);
    return ageMin < 60
      ? `${ageMin}m ago`
      : ageMin < 1440
        ? `${Math.round(ageMin / 60)}h ago`
        : `${Math.round(ageMin / 1440)}d ago`;
  }

  // HA static-file mirror written by the irrigoto_mirror_csv_after_
  // watering automation (see homeassistant/packages/irrigoto.yaml).
  // These files survive device sleep so the card can render without
  // waking it. Filenames are 0-based zone_id, matching the card's
  // own zone_id config.
  _haFilePaths() {
    const id = this._config.zone_id;
    // Per-device sub-folder keeps multiple devices' caches from clobbering
    // each other. Falls back to the legacy flat path if no slug is known.
    const slug = this._config.mirror_slug;
    const base = slug
      ? `/local/community/irrigoto/${slug}/csv`
      : `/local/community/irrigoto/csv`;
    return {
      csv: `${base}/zone_${id}.csv`,
      lastWater: `${base}/zone_${id}_lastwater.json`,
      apiAll: `${base}/api_all.json`,
    };
  }

  // HEAD request to get the file's Last-Modified header as a ms
  // timestamp. Returns 0 if the file doesn't exist or the header is
  // missing. Used to compare against the HA sensor's last_finished
  // timestamp so we know whether to bother the device for fresher
  // data or just use the mirrored file.
  async _fileLastModified(url) {
    try {
      const r = await fetch(url, { method: "HEAD", cache: "no-store" });
      if (!r.ok) return 0;
      const lm = r.headers.get("Last-Modified");
      return lm ? Date.parse(lm) || 0 : 0;
    } catch (e) {
      return 0;
    }
  }

  // Returns the HA last_finished sensor ms for this zone (or 0 if
  // not available). HA sensors are 1-based (zone_1, zone_2, …) while
  // card's zone_id is 0-based.
  _sensorLastFinishedMs() {
    if (!this._hass) return 0;
    const entity = `sensor.irrigoto_zone_${this._config.zone_id + 1}_last_finished`;
    const s = this._hass.states[entity];
    if (!s || s.state === "unknown" || s.state === "unavailable") return 0;
    return Date.parse(s.state) || 0;
  }

  // Compare two zone objects (from /api/all) for "same zone
  // definition." If polygon point count or any vertex moved
  // significantly, the zone was edited and old CSV data no longer
  // makes geometric sense -- the cached heat would visually
  // mismatch the new polygon outline. Small floating-point wobble
  // (sub-degree, sub-mm) doesn't count as a change.
  _zonesMatch(a, b) {
    if (!a || !b || !a.points || !b.points) return false;
    if (a.points.length !== b.points.length) return false;
    for (let i = 0; i < a.points.length; i++) {
      const p = a.points[i], q = b.points[i];
      if (Math.abs((p.deg || 0) - (q.deg || 0)) > 0.5) return false;
      if (Math.abs((p.mm || 0) - (q.mm || 0)) > 10) return false;
      if ((p.widx ?? -1) !== (q.widx ?? -1)) return false;
    }
    return true;
  }

  // Fetch the heatmap inputs from the device directly. Zone polygon
  // must succeed; lastWater + csvText come back raw (possibly empty
  // if the device has purged old data). Fallback layering happens in
  // _layerCacheIfIncomplete, which runs after any source fetch.
  async _fetchFromDevice(force) {
    if (force) this._resolvedUrl = null;
    const base = await this._resolveDeviceUrl();
    const zoneId = this._config.zone_id;
    const [apiAll, lastWater] = await Promise.all([
      this._fetchJson(`${base}/api/all`),
      this._fetchJson(`${base}/zone/last_water?id=${zoneId}`),
    ]);
    const zone = (apiAll.zones || []).find((z) => z.id === zoneId);
    if (!zone) throw new Error(`zone ${zoneId} not on device`);
    let csvText = "";
    try {
      csvText = await this._fetchText(`${base}/zone/water_csv?id=${zoneId}`);
    } catch (e) {
      // Common when device purges old wbin files to free space.
      // _layerCacheIfIncomplete will fill in from cache.
    }
    return { zone, lastWater, csvText, source: "device" };
  }

  // Fetch from the HA-mirrored static files. apiAll must be present;
  // lastWater + csvText come back raw and any gaps are layered in by
  // _layerCacheIfIncomplete.
  async _fetchFromHAFile() {
    const p = this._haFilePaths();
    const zoneId = this._config.zone_id;
    const apiAll = await this._fetchJson(p.apiAll);
    const zone = (apiAll.zones || []).find((z) => z.id === zoneId);
    if (!zone) throw new Error(`zone ${zoneId} not in HA-mirrored api_all`);
    let lastWater = { rings: [] };
    let csvText = "";
    try { lastWater = await this._fetchJson(p.lastWater); } catch {}
    try { csvText = await this._fetchText(p.csv); } catch {}
    return { zone, lastWater, csvText, source: "hafile" };
  }

  // Layer cached data into a fetch result for fields that came back
  // empty. Device routinely purges old wbin files (which empties
  // lastWater.rings AND removes the CSV) when free space runs low.
  // Without this layering, the card would show "no watering data
  // yet" the moment housekeeping kicks in, even though the HA
  // mirror and localStorage cache still have the last-good
  // snapshot. Goes through fallback sources in order:
  //   1. The OTHER static source (HA mirror if result came from
  //      device, device URL if result came from HA mirror -- but
  //      that second case is rare and we only do one direction)
  //   2. localStorage cache, gated on polygon match so an edited
  //      zone doesn't show geometrically-stale heat
  async _layerCacheIfIncomplete(result) {
    const noRings = !(result.lastWater?.rings?.length);
    const noCsv = !result.csvText;
    if (!noRings && !noCsv) return result; // complete

    // From device path: try the HA file mirror to fill gaps.
    if (result.source === "device") {
      const p = this._haFilePaths();
      if (noRings) {
        try {
          const lw = await this._fetchJson(p.lastWater);
          if (lw?.rings?.length) {
            console.log(
              "[irrigoto-heatmap-card] zone %d: device returned empty rings; using HA mirror lastWater",
              this._config.zone_id,
            );
            result.lastWater = lw;
          }
        } catch {}
      }
      if (!result.csvText) {
        try {
          const t = await this._fetchText(p.csv);
          if (t) {
            console.log(
              "[irrigoto-heatmap-card] zone %d: device CSV missing; using HA mirror CSV",
              this._config.zone_id,
            );
            result.csvText = t;
          }
        } catch {}
      }
    }

    // Still incomplete? Fall back to localStorage, but only if its
    // stored zone polygon still matches the current device polygon.
    // An edited zone invalidates the cached snapshot geometrically.
    const stillNoRings = !(result.lastWater?.rings?.length);
    const stillNoCsv = !result.csvText;
    if (stillNoRings || stillNoCsv) {
      const cached = this._loadCache();
      if (cached && this._zonesMatch(result.zone, cached.zone)) {
        if (stillNoRings && cached.lastWater?.rings?.length) {
          console.log(
            "[irrigoto-heatmap-card] zone %d: using cached lastWater from localStorage",
            this._config.zone_id,
          );
          result.lastWater = cached.lastWater;
        }
        if (stillNoCsv && cached.csvText) {
          console.log(
            "[irrigoto-heatmap-card] zone %d: using cached csvText from localStorage",
            this._config.zone_id,
          );
          result.csvText = cached.csvText;
        }
      } else if (cached) {
        console.log(
          "[irrigoto-heatmap-card] zone %d: localStorage cache exists but zone polygon was edited; skipping stale cache",
          this._config.zone_id,
        );
      }
    }

    return result;
  }

  async _loadData(force) {
    if (this._loading) return;
    this._loading = true;

    // Render cached snapshot immediately (if available) so the user
    // doesn't stare at "Loading…" for the full network timeout when
    // the device is asleep/offline. If the live fetch then succeeds,
    // we override; if it fails, the cached view stays put with the
    // age badge updated.
    let renderedFromCache = false;
    if (!force) {
      const cached = this._loadCache();
      if (cached && cached.zone) {
        this._zone = cached.zone;
        this._lastRun = cached.lastWater;
        this._csvRows = this._parseCsv(cached.csvText || "");
        this._patchTitle();
        this._draw();
        this._renderInfo();
        this._showStatus(`cached • ${this._ageStr(cached.ts)}`, false, true);
        renderedFromCache = true;
      } else {
        this._showStatus("Loading…");
      }
    } else {
      this._showStatus("Loading…");
    }

    // Source-selection: decide whether the device likely has fresher
    // data than HA's mirrored file. If so, prefer device (and fall
    // back to file on failure). Otherwise prefer the file (no need
    // to bother a sleeping device for data we already have).
    //
    //   force-refresh        → device first
    //   no HA file exists    → device first
    //   sensor > file + 60s  → device first (device just watered,
    //                          mirror automation may not have caught
    //                          up yet)
    //   else                 → file first (sleeping device, nothing
    //                          new to fetch)
    const sensorMs = this._sensorLastFinishedMs();
    const fileMs = force ? 0 : await this._fileLastModified(this._haFilePaths().csv);
    let preferDevice = force || fileMs === 0;
    if (!preferDevice && sensorMs > fileMs + 60_000) preferDevice = true;
    const sources = preferDevice
      ? [this._fetchFromDevice.bind(this, force), this._fetchFromHAFile.bind(this)]
      : [this._fetchFromHAFile.bind(this), this._fetchFromDevice.bind(this, force)];

    let result = null;
    let lastErr = null;
    for (const src of sources) {
      try {
        result = await src();
        break;
      } catch (e) {
        lastErr = e;
        console.log(
          "[irrigoto-heatmap-card] zone %d source attempt failed (%s); trying next",
          this._config.zone_id,
          e.message,
        );
      }
    }

    try {
      if (!result) throw lastErr || new Error("no data sources available");
      // Fill in any empty pieces (rings, csv) from cached sources.
      // The fetch sources return raw -- empty when the device has
      // purged old data -- and this step layers fallback content in.
      result = await this._layerCacheIfIncomplete(result);
      const { zone, lastWater, csvText, source } = result;
      this._zone = zone;
      this._lastRun = lastWater;
      this._csvRows = this._parseCsv(csvText);
      this._patchTitle();
      console.log(
        "[irrigoto-heatmap-card] zone %d: source=%s rings=%d csv_rows=%d (raw_text_bytes=%d)",
        this._config.zone_id,
        source,
        (lastWater.rings || []).length,
        this._csvRows.length,
        csvText.length,
      );
      if (csvText.length > 0 && this._csvRows.length === 0) {
        console.log(
          "[irrigoto-heatmap-card] zone %d csv preview (first 300 chars):\n%s",
          this._config.zone_id,
          csvText.slice(0, 300),
        );
      }
      if (this._csvRows.length > 0) {
        this._saveCache(zone, lastWater, csvText);
      }
      this._draw();
      this._renderInfo();
      // Tag the status to indicate where the data came from. Live
      // device fetch → clean status. HA file → small "mirrored •
      // age" badge so user knows it's not freshly off the device.
      if (source === "hafile") {
        const ageStr = fileMs > 0 ? ` • ${this._ageStr(fileMs)}` : "";
        this._showStatus(`mirrored${ageStr}`, false, true);
      } else {
        this._showStatus("");
      }
    } catch (err) {
      console.error("[irrigoto-heatmap-card]", err);
      if (renderedFromCache) {
        // Already showing cached snapshot; leave it.
      } else {
        const cached = this._loadCache();
        if (cached && cached.zone) {
          this._zone = cached.zone;
          this._lastRun = cached.lastWater;
          this._csvRows = this._parseCsv(cached.csvText || "");
          this._patchTitle();
          this._draw();
          this._renderInfo();
          this._showStatus(`cached • ${this._ageStr(cached.ts)}`, false, true);
        } else {
          this._showStatus(`Error: ${err.message}`, true);
        }
      }
    } finally {
      this._loading = false;
    }
  }

  _fetchJson(url) {
    return fetch(url, { cache: "no-store" }).then((r) => {
      if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
      return r.json();
    });
  }

  _fetchText(url) {
    return fetch(url, { cache: "no-store" }).then((r) => {
      if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
      return r.text();
    });
  }

  _parseCsv(txt) {
    const lines = (txt || "").trim().split(/\r?\n/);
    if (lines.length < 2) return [];
    const hdr = lines[0].split(",");
    const idx = (k) => hdr.indexOf(k);
    if (idx("nozzle_deg_actual") < 0) return [];
    // time_s / pass_type may not exist in legacy CSVs; -1 -> ignored.
    // For smooth-aggregate rows (the format that current builds write),
    // the firmware overloads time_ds to carry per-ring deposited depth
    // (depth_mm * 100) and marks pass_type = 255. The CSV emitter writes
    // time_ds * 0.1 as time_s, so CSV time_s = depth_mm * 10 for these
    // rows. The card recovers deposited depth via time_s / 10 instead
    // of running the flow/dps formula (which only gives single-pass).
    const iTime = idx("time_s");
    const iPass = idx("pass_type");
    const out = [];
    for (let i = 1; i < lines.length; i++) {
      const c = lines[i].split(",");
      if (!c.length) continue;
      const rawActMm = Number(c[idx("throw_mm_actual")]);
      if (!(rawActMm > 100)) continue;
      const tgtMm = Number(c[idx("throw_mm_target")]);
      // Low-PSI position guard: below ~1.5 PSI the pressure->throw model
      // over-reads badly (up to 2x on inner rings), so the "actual" radius
      // is artifact, not landing position. Serpentine runs surfaced this:
      // it retires rings on DEPTH, so the position residue survives into
      // the aggregate and the heat piled into the mid-radius band (smooth
      // masks it by re-firing rings until measured throw is within 20%).
      // Trust the measured radius only within +/-25% of the ring target;
      // beyond that, render at the target radius. Depth/color unaffected.
      let actMm = rawActMm;
      if (tgtMm > 100) {
        const ratio = rawActMm / tgtMm;
        if (ratio < 0.75 || ratio > 1.25) actMm = tgtMm;
      }
      out.push({
        ring: Number(c[idx("ring")]),
        sector: Number(c[idx("sector")]),
        actDeg: Number(c[idx("nozzle_deg_actual")]),
        tgtDeg: Number(c[idx("nozzle_deg_target")]),
        actMm,
        rawActMm,
        tgtMm,
        psiAct: Number(c[idx("pressure_psi_actual")]),
        psiTgt: Number(c[idx("pressure_psi_target")]),
        timeS: iTime >= 0 ? Number(c[iTime]) : NaN,
        passType: iPass >= 0 ? Number(c[iPass]) : NaN,
      });
    }
    return out;
  }

  _showStatus(text, isError, isCached) {
    this._status.textContent = text || "";
    this._status.classList.toggle("error", !!isError);
    this._status.classList.toggle("cached", !!isCached);
  }

  _renderInfo() {
    if (!this._lastRun) {
      this._info.innerHTML = "";
      return;
    }
    const r = this._lastRun;
    // Per-zone watering stats live in HA template sensors (defined in
    // packages/irrigoto.yaml). They're 1-based (sensor.irrigoto_zone_1_*
    // etc.) while card's zone_id is 0-based, so +1 for the lookup.
    const zid = this._config.zone_id + 1;
    // Track which sensors came back null so we can surface that
    // diagnostic in the console -- otherwise it looks like the card
    // is broken when really HA hasn't populated the trigger-based
    // template sensors yet (no event fired since YAML reload).
    const sensorStatus = {};
    const stateOf = (suffix) => {
      const eid = `sensor.irrigoto_zone_${zid}_last_${suffix}`;
      if (!this._hass) { sensorStatus[suffix] = "no-hass"; return null; }
      const e = this._hass.states[eid];
      if (!e) { sensorStatus[suffix] = "missing"; return null; }
      if (e.state === "unknown" || e.state === "unavailable") {
        sensorStatus[suffix] = e.state;
        return null;
      }
      sensorStatus[suffix] = "ok";
      return e.state;
    };

    const parts = [];

    // ── Per-zone last-watering stats (from HA sensors) ───────────
    const finished = stateOf("finished");
    if (finished) {
      const ts = Date.parse(finished);
      if (ts) {
        // Compact, locale-aware date + time. Examples (en-US):
        //   "May 19, 2:30 PM"   "Jan 3, 9:05 AM"
        // Examples (en-GB, de-DE):
        //   "19 May, 14:30"     "19. Mai, 14:30"
        // Tooltip carries both the full datetime and the relative
        // age so hovering tells you "10h ago".
        const dt = new Date(ts);
        const dtStr = new Intl.DateTimeFormat(undefined, {
          month: "short",
          day: "numeric",
          hour: "numeric",
          minute: "2-digit",
        }).format(dt);
        const tip = `${dt.toLocaleString()} • ${this._ageStr(ts)}`;
        parts.push(`<span title="${tip}">when <b>${dtStr}</b></span>`);
      }
    }
    const status = stateOf("status");
    if (status) parts.push(`<span>status <b>${status}</b></span>`);
    const score = stateOf("score");
    if (score) parts.push(`<span>score <b>${Number(score).toFixed(1)}/5</b></span>`);
    const volume = stateOf("volume");
    if (volume) parts.push(`<span>vol <b>${Number(volume).toFixed(1)} L</b></span>`);
    const duration = stateOf("duration");
    if (duration) parts.push(`<span>dur <b>${Number(duration).toFixed(1)} min</b></span>`);
    const coverage = stateOf("coverage_ratio");
    if (coverage) parts.push(`<span>cov <b>${Number(coverage).toFixed(0)}%</b></span>`);

    // ── Per-run ring/supply stats (from /zone/last_water JSON) ────
    parts.push(`<span>rings <b>${r.num_rings ?? "?"}</b></span>`);
    parts.push(`<span>supply <b>${(r.supply_psi_min ?? 0).toFixed(1)}–${(r.supply_psi_max ?? 0).toFixed(1)} psi</b></span>`);
    if ((r.rings_supply_limited ?? 0) > 0) {
      parts.push(`<span class="flag-warn">flagged <b>${r.rings_supply_limited}</b></span>`);
    }
    parts.push(`<span class="dim">build <b>${r.fw_build ?? "?"}</b></span>`);

    this._info.innerHTML = parts.join("");

    // Diagnostic: log per-zone sensor status, but only when it changes
    // since the last render (else we'd flood the console because
    // _renderInfo now runs on every hass update).
    const statusKey = Object.entries(sensorStatus)
      .map(([k, v]) => `${k}=${v}`)
      .join(",");
    if (statusKey !== this._lastSensorStatusKey) {
      this._lastSensorStatusKey = statusKey;
      const notOk = Object.entries(sensorStatus).filter(([, v]) => v !== "ok");
      if (notOk.length) {
        console.log(
          "[irrigoto-heatmap-card] zone %d sensor states changed:",
          this._config.zone_id,
          sensorStatus,
        );
      }
    }
  }

  _draw() {
    const canvas = this._canvas;
    if (!canvas) return;
    // Canvas backing pixels MUST equal the actual device pixels the
    // canvas occupies on screen -- otherwise the browser bilinearly
    // upscales the backing during display, smoothing heat pixels with
    // their transparent neighbors and dropping their effective alpha.
    // That dilution is the root cause of the long-running "card looks
    // dim vs. device web UI" mismatch: prior attempts set backing =
    // cssSize (CSS px, ignoring devicePixelRatio), so on retina/HiDPI
    // displays the browser still upscaled 2-3x to device pixels with
    // bilinear smoothing.
    //
    // Fix: backing = cssSize * DPR, and we explicitly lock the CSS
    // size with canvas.style.width/height so any later layout
    // recomputation doesn't change the display->backing ratio.
    //
    // If layout hasn't settled yet (clientWidth < 50, which can
    // happen mid-mount), defer one rAF tick and try again.
    const cssSize = canvas.clientWidth;
    if (cssSize < 50) {
      if (!this._drawDeferred) {
        this._drawDeferred = true;
        requestAnimationFrame(() => {
          this._drawDeferred = false;
          this._draw();
        });
      }
      return;
    }
    const dpr = window.devicePixelRatio || 1;
    const pix = Math.round(cssSize * dpr);
    if (canvas.width !== pix) canvas.width = pix;
    if (canvas.height !== pix) canvas.height = pix;
    canvas.style.width = cssSize + "px";
    canvas.style.height = cssSize + "px";
    const ctx = canvas.getContext("2d");
    const W = pix;
    const H = pix;
    const cx = W / 2;
    const cy = H / 2;
    // Symmetric inset large enough to fully clear the slim right-edge
    // legend overlay (~16 CSS px wide: 5px bar + 5px labels + 5px
    // caption + gaps). 22 backing px (~18 CSS px) gives a small
    // visible margin between the radar and the legend. Radar stays
    // perfectly centered horizontally.
    const maxR = cx - 22 * dpr;
    console.log(
      "[irrigoto-heatmap-card] canvas: backing=%dx%d, css=%dx%d, dpr=%s",
      W, H, cssSize, cssSize, dpr
    );

    // Match the device's zone_setup background stack exactly: dark
    // green radial gradient inside the maxR circle, range rings,
    // and an orange-tinted polygon fill. The heatmap renders ON TOP
    // of this stack. The 9% orange polygon tint + dark green
    // background warm up the heat colors so the same depth-derived
    // RGBA values read much richer than they would against a flat
    // dark card background -- this is the missing visual depth.
    ctx.clearRect(0, 0, W, H);

    // Dark green radial gradient inside maxR
    const bg = ctx.createRadialGradient(cx, cy, 0, cx, cy, maxR);
    bg.addColorStop(0, "#0b1e16");
    bg.addColorStop(1, "#050c08");
    ctx.beginPath();
    ctx.arc(cx, cy, maxR, 0, Math.PI * 2);
    ctx.fillStyle = bg;
    ctx.fill();

    // Bearing spokes every 30 degrees (very subtle)
    ctx.save();
    ctx.strokeStyle = "#07180e";
    ctx.lineWidth = 0.6 * dpr;
    for (let a = 0; a < 360; a += 30) {
      const rad = (a * Math.PI) / 180;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + Math.cos(rad) * maxR, cy + Math.sin(rad) * maxR);
      ctx.stroke();
    }
    ctx.restore();

    // Range rings -- darker green like the device, subtler than grid
    ctx.save();
    // Display domain (mm at rim): larger of the configured max throw and the
    // zone's own farthest point, + 3ft (914mm) buffer. Auto-fits any zone at
    // any supply pressure while respecting the configured reach as a floor.
    const _zpts = (this._zone && this._zone.points) || [];
    const _zmax = _zpts.length ? Math.max(..._zpts.map((p) => p.mm || 0)) : 0;
    const displayMax = Math.max(this._config.max_throw_mm, _zmax) + 914;
    const maxFtScale = displayMax / 304.8;
    // Range-ring ticks derived from the domain so the grid adapts to pressure.
    const _baseFt = Math.max(this._config.max_throw_mm, _zmax) / 304.8;
    const _ringStep = _baseFt > 40 ? 10 : 5;
    const ftValues = [];
    for (let f = _ringStep; f <= _baseFt + 0.01; f += _ringStep) ftValues.push(f);
    ftValues.forEach((ft) => {
      const r = (ft / maxFtScale) * maxR;
      if (r > maxR) return;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.strokeStyle = "#081e10";
      ctx.lineWidth = 0.7 * dpr;
      ctx.stroke();
    });
    ctx.restore();

    // Render whenever CSV rows are present, regardless of whether
    // lastWater.rings is populated. The device routinely purges the
    // per-ring water_run_t blob (used to fill rings[]) when free
    // space gets low, even when the CSV file's data is still
    // available in the HA mirror or localStorage. The CSV itself
    // carries everything needed for smooth-aggregate rendering
    // (per-row depth via time_s/10); LR[ring-1].dps is only used
    // for the legacy formula path, which falls back to dps=1.0
    // when LR is empty.
    const hasCsv = this._csvRows && this._csvRows.length > 0;
    const hasRings = this._lastRun?.rings?.length > 0;
    if (!hasCsv && !hasRings) {
      this._drawCenterText(ctx, cx, cy, "No watering data yet", dpr);
      return;
    }
    if (!hasCsv) {
      this._drawCenterText(ctx, cx, cy, "No CSV data — water the zone first", dpr);
      return;
    }

    const LR = (this._lastRun && this._lastRun.rings) || [];
    // Normalize the color ramp to the run's actual target depth (N x 1/8",
    // firmware b388+) so a deeper run reads on-target rather than "over".
    // Fall back to the configured default for pre-b388 runs (field absent/0).
    const runTarget = this._lastRun && Number(this._lastRun.target_depth_mm);
    const targetDepth = (runTarget && runTarget > 0)
      ? runTarget
      : this._config.target_depth_mm;
    // Display scale (mm at rim): the dynamic domain computed above.
    const maxThrow = displayMax;

    // Build per-ring throw averages from CSV
    const ringThrowMap = {};
    this._csvRows.forEach((r) => {
      if (!ringThrowMap[r.ring]) ringThrowMap[r.ring] = { sum: 0, n: 0 };
      ringThrowMap[r.ring].sum += r.actMm;
      ringThrowMap[r.ring].n++;
    });
    const sortedRings = Object.keys(ringThrowMap)
      .map(Number)
      .sort((a, b) => a - b);
    const ringAvgMm = {};
    sortedRings.forEach((r) => {
      ringAvgMm[r] = ringThrowMap[r].sum / ringThrowMap[r].n;
    });

    // Zone polygon FILL (rgba(255,140,0, 0.09)) BEFORE the heatmap.
    // This is the device's pre-heatmap step that warms up the heat
    // colors -- heatmap pixels alpha-composite on top of the orange
    // tint so the same RGBA depth-derived colors read warmer. The
    // polygon outline is drawn later, ON TOP of the heatmap.
    const polyPts = (this._zone && this._zone.points) || [];
    if (polyPts.length >= 3) {
      ctx.save();
      ctx.beginPath();
      polyPts.forEach((p, i) => {
        const r = (p.mm / maxThrow) * maxR;
        const a = ((p.deg - 90) * Math.PI) / 180;
        const x = cx + Math.cos(a) * r;
        const y = cy + Math.sin(a) * r;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(255,140,0,0.09)";
      ctx.fill();
      ctx.strokeStyle = "rgba(255,140,0,0.55)";
      ctx.lineWidth = 1.5 * dpr;
      ctx.stroke();
      ctx.restore();
    }

    // Per-sector pixel heatmap -- 1:1 port of the device's
    // zone_setup.html actual-mode renderer (lines 515-589). Each CSV
    // row contributes single-pass depth into a Float32 depth buffer
    // over its (annulus, sector) bounding box. Pixels hit by multiple
    // rows (CW+CCW overlap) accumulate, so doubly-covered pixels
    // reach ~2x the single-pass target depth and color amber/red.
    // The color ramp matches the device exactly.
    //
    // Spread: the device uses ctx.filter='blur(3px)' for organic
    // softening. Repeated efforts to match the device's vibrancy with
    // blur in place failed -- the heat consistently renders dimmer
    // than the device. Replaced with a 5-tap offset-draw blur
    // (center + 4 cardinal neighbors at DPR-scaled distance) that
    // operates on the source canvas, sidestepping any ctx.filter
    // shadow-DOM/compositing quirks.
    const LOG_DEG = 2.0;
    const BW = W;
    const BH = H;
    const depthBuf = new Float32Array(BW * BH);

    let maxDepth = 0;
    let drawn = 0;
    let smoothAggrCount = 0;
    this._csvRows.forEach((row) => {
      const { ring, actDeg, actMm, psiAct, timeS, passType } = row;
      const ri = sortedRings.indexOf(ring);
      if (ri < 0) return;
      const rawO =
        ri === 0
          ? actMm * 1.08
          : (ringAvgMm[sortedRings[ri - 1]] + actMm) / 2;
      const rawI =
        ri === sortedRings.length - 1
          ? actMm * 0.92
          : (actMm + ringAvgMm[sortedRings[ri + 1]]) / 2;

      // Smooth-aggregate rows (pass_type === 255) carry the firmware's
      // tracked deposited depth in the time_s column (depth_mm * 10).
      // Match the device's actual-mode renderer exactly (see
      // main/zone_setup_html.h:592-640): use totalDepth raw, wedge at
      // 10° to match WATER_SECTOR_DEG, and accumulate with += into
      // depthBuf. The += is intentional -- adjacent rings' wedges
      // share a 1-pixel-wide radial boundary, so at those pixels two
      // ring_total depths sum (2x ratio -> red), producing the bright
      // boundary stripes that are the "radial detail" in the device's
      // view. Inside each ring the wedges of different sectors don't
      // overlap angularly, so they don't double-count.
      let depth;
      let wedgeDeg = LOG_DEG;
      if (passType === 255 && Number.isFinite(timeS) && timeS > 0) {
        depth = timeS / 10;
        wedgeDeg = 10;
        smoothAggrCount++;
      } else {
        const dps =
          LR && LR[ring - 1] && LR[ring - 1].dps ? LR[ring - 1].dps : 1.0;
        const flowLpm = 0.12 * ((psiAct / 5.034) * 100) + 1.0;
        const r_m = actMm / 1000;
        // Inside the ellipse-splash threshold the firmware's dps solver sizes
        // the sweep for a fixed ~300mm-wide splash band, not the sub-100mm
        // geometric ring gap (mirrors firmware smooth_display_depth_mm /
        // irrigoto.c ~line 9279). Dividing the same water by the narrow
        // geometric annulus inflates depth 2-4x -- the bogus "hot core".
        // Use the splash band here too so per-pass (pulse/cleanup) rows read
        // true depth. (Smooth/255 rows already carry firmware-corrected depth
        // above.) Painting geometry (rawO/rawI) is left unchanged.
        const SPLASH_THRESH_MM = 1829;
        const SPLASH_MM = 300;
        const dr_m =
          actMm < SPLASH_THRESH_MM ? SPLASH_MM / 1000 : (rawO - rawI) / 1000;
        const area_m2 =
          Math.PI *
          ((r_m + dr_m / 2) ** 2 - (r_m - dr_m / 2) ** 2) *
          (LOG_DEG / 360);
        depth =
          ((flowLpm / 60 / 1000) * (LOG_DEG / dps)) /
          Math.max(area_m2, 1e-6) *
          1000;
      }

      const bearLo = actDeg - wedgeDeg / 2;
      const bearHi = actDeg + wedgeDeg / 2;
      const rOPx = (rawO / maxThrow) * maxR;
      const rIPx = Math.max(0, (rawI / maxThrow) * maxR);

      const cRad = ((actDeg - 90) * Math.PI) / 180;
      const midRPx = (rOPx + rIPx) / 2;
      const halfRad = (rOPx - rIPx) / 2 + 2;
      // Bounding-box half-angle: use the OUTER radius so the box covers
      // the full wedge extent even when LOG_DEG=10 makes wider arcs.
      const halfAng =
        rOPx * Math.sin(((wedgeDeg / 2) * Math.PI) / 180) + 2;
      const bcx = cx + midRPx * Math.cos(cRad);
      const bcy = cy + midRPx * Math.sin(cRad);
      const box = halfRad + halfAng;
      const x0 = Math.max(0, Math.floor(bcx - box));
      const x1 = Math.min(BW - 1, Math.ceil(bcx + box));
      const y0 = Math.max(0, Math.floor(bcy - box));
      const y1 = Math.min(BH - 1, Math.ceil(bcy + box));

      for (let px = x0; px <= x1; px++) {
        for (let py = y0; py <= y1; py++) {
          const dx = px - cx;
          const dy = py - cy;
          const r2 = Math.sqrt(dx * dx + dy * dy);
          if (r2 < rIPx || r2 > rOPx) continue;
          const bear = ((Math.atan2(dx, -dy) * 180) / Math.PI + 360) % 360;
          let inSec;
          if (bearHi >= 360) inSec = bear >= bearLo || bear <= bearHi - 360;
          else if (bearLo < 0) inSec = bear >= bearLo + 360 || bear <= bearHi;
          else inSec = bear >= bearLo && bear <= bearHi;
          if (inSec) {
            const idx = py * BW + px;
            // Accumulate -- matches device. For smooth-aggregate this
            // only doubles at ring-boundary pixels (since 10° wedges
            // don't overlap angularly within a ring), producing the
            // bright radial boundary stripes visible in the device's
            // view. For per-pass rows it stacks CW+CCW genuine
            // overlap.
            depthBuf[idx] += depth;
            if (depthBuf[idx] > maxDepth) maxDepth = depthBuf[idx];
          }
        }
      }
      drawn++;
    });

    // Color map: identical ramp to zone_setup.html. Alpha rises
    // proportionally with ratio from 0 to 200 over the blue->teal
    // span and stays high (200-240) in the green->red bands.
    const imgData = ctx.createImageData(BW, BH);
    const d32 = imgData.data;
    let painted = 0;
    const ratioBins = [0, 0, 0, 0, 0]; // <0.6, <0.88, <1.12, <1.5, >=1.5
    for (let i = 0; i < BW * BH; i++) {
      const dep = depthBuf[i];
      if (dep < 0.005) continue;
      const ratio = dep / targetDepth;
      let r = 0, g = 0, b2 = 0, a = 0;
      if (ratio < 0.6) {
        r = 40;
        g = 90 + Math.round((ratio / 0.6) * 80);
        b2 = 200;
        a = Math.round((ratio / 0.6) * 200);
        ratioBins[0]++;
      } else if (ratio < 0.88) {
        const t = (ratio - 0.6) / 0.28;
        r = 40;
        g = 170 + Math.round(t * 50);
        b2 = Math.round(200 * (1 - t));
        a = 200;
        ratioBins[1]++;
      } else if (ratio < 1.12) {
        const t = (ratio - 0.88) / 0.24;
        r = Math.round(40 + t * 120);
        g = 220;
        b2 = 0;
        a = 220;
        ratioBins[2]++;
      } else if (ratio < 1.5) {
        const t = (ratio - 1.12) / 0.38;
        r = 160 + Math.round(t * 80);
        g = Math.round(220 * (1 - t * 0.5));
        b2 = 0;
        a = 230;
        ratioBins[3]++;
      } else {
        r = 240;
        g = Math.max(0, 80 - Math.round((ratio - 1.5) * 40));
        b2 = 0;
        a = 240;
        ratioBins[4]++;
      }
      const base = i * 4;
      d32[base] = r;
      d32[base + 1] = g;
      d32[base + 2] = b2;
      d32[base + 3] = Math.min(255, a);
      painted++;
    }

    // Match the device's exact blur: 3px at backing 330 = 0.909% of
    // canvas. Scale by current backing so the relative softening is
    // the same across display densities / card sizes.
    const tmpC = document.createElement("canvas");
    tmpC.width = BW;
    tmpC.height = BH;
    tmpC.getContext("2d").putImageData(imgData, 0, 0);
    const blurRadius = Math.max(1, Math.round((BW * 3) / 330));
    ctx.save();
    ctx.filter = `blur(${blurRadius}px)`;
    ctx.drawImage(tmpC, 0, 0);
    ctx.restore();

    // Template-literal log so float precision actually renders (Chrome's
    // console.log doesn't honor %.3f/%.2f precision specifiers in the
    // printf-style API -- they show as literal text and the values get
    // pushed into the wrong fields. Build the string up front instead).
    const zid = this._config.zone_id;
    const sampleRow = this._csvRows[0] || null;
    const dpsList = (LR || []).map((r) => (r && r.dps !== undefined ? r.dps : "?"));
    const psiList = (LR || []).map((r) => (r && r.psi !== undefined ? r.psi : "?"));
    console.log(
      `[irrigoto-heatmap-card] zone ${zid}: ${drawn} wedges (${smoothAggrCount} smooth-aggr), ${painted}/${BW * BH} lit, max=${maxDepth.toFixed(3)}mm ratio=${(maxDepth / targetDepth).toFixed(2)}x, target=${targetDepth.toFixed(3)}mm, bins[<.6/<.88/<1.12/<1.5/>=1.5]=[${ratioBins.join(",")}]`,
    );
    console.log(
      `[irrigoto-heatmap-card] zone ${zid} dps per ring (1..N):`,
      dpsList.join(","),
    );
    if (sampleRow) {
      // Recompute depth for the first CSV row so we can see what one
      // wedge contributes. If this single-row depth is well below
      // target, the formula is producing low values; if it's near
      // target, the issue is overlap/accumulation, not per-row depth.
      const { ring, actDeg, actMm, psiAct, timeS, passType } = sampleRow;
      const ri = sortedRings.indexOf(ring);
      const rawO_s = ri === 0 ? actMm * 1.08 : (ringAvgMm[sortedRings[ri - 1]] + actMm) / 2;
      const rawI_s = ri === sortedRings.length - 1 ? actMm * 0.92 : (actMm + ringAvgMm[sortedRings[ri + 1]]) / 2;
      const dps_s = LR && LR[ring - 1] && LR[ring - 1].dps ? LR[ring - 1].dps : 1.0;
      const flow_s = 0.12 * ((psiAct / 5.034) * 100) + 1.0;
      const r_m_s = actMm / 1000;
      const dr_m_s = (rawO_s - rawI_s) / 1000;
      const area_s = Math.PI * ((r_m_s + dr_m_s / 2) ** 2 - (r_m_s - dr_m_s / 2) ** 2) * (LOG_DEG / 360);
      const dep_formula = ((flow_s / 60 / 1000) * (LOG_DEG / dps_s)) / Math.max(area_s, 1e-6) * 1000;
      const dep_smooth = (passType === 255 && Number.isFinite(timeS)) ? timeS / 10 : NaN;
      const dep_used = Number.isFinite(dep_smooth) && dep_smooth > 0 ? dep_smooth : dep_formula;
      console.log(
        `[irrigoto-heatmap-card] zone ${zid} sample row: ring=${ring} actDeg=${actDeg} actMm=${actMm} psiAct=${psiAct} passType=${passType} timeS=${timeS} | formula depth=${dep_formula.toFixed(3)}mm (ratio ${(dep_formula / targetDepth).toFixed(2)}x), smooth-aggr depth=${Number.isFinite(dep_smooth) ? dep_smooth.toFixed(3) : "n/a"}mm (ratio ${Number.isFinite(dep_smooth) ? (dep_smooth / targetDepth).toFixed(2) : "n/a"}x) -> USING ${dep_used.toFixed(3)}mm`,
      );
    }

    // Range labels along the East axis (5', 10', 15', 20', 25').
    // Matches the on-device zone setup view: faint gray text at each
    // gridline radius, in feet. Helps gauge scale without studying
    // the heat color alone.
    ctx.save();
    ctx.font = `${Math.max(9 * dpr, W * 0.022)}px sans-serif`;
    ctx.fillStyle = "rgba(150,170,190,0.55)";
    ctx.textBaseline = "middle";
    ctx.textAlign = "left";
    // Label each grid ring at its actual radius so labels and rings align.
    ftValues.forEach((ft) => {
      const ringR = (ft / maxFtScale) * maxR;
      if (ringR > maxR) return;
      ctx.fillText(`${ft}'`, cx + ringR + 4 * dpr, cy);
    });
    ctx.restore();

    // Zone polygon outline + numbered vertex markers. Mirrors what the
    // device's own zone_setup view shows: green polygon line, with a
    // small filled circle at each perimeter point labeled by widx
    // (the walk index from the calibration walk). The numbered dots
    // were what made the device heatmap look "hot" at the corners --
    // they're decorations, not actual depth, and reproducing them
    // here keeps the visualization consistent.
    const pts = (this._zone && this._zone.points) || [];
    if (pts.length >= 3) {
      ctx.save();
      ctx.beginPath();
      pts.forEach((p, i) => {
        const r = (p.mm / maxThrow) * maxR;
        const a = ((p.deg - 90) * Math.PI) / 180;
        const x = cx + Math.cos(a) * r;
        const y = cy + Math.sin(a) * r;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.strokeStyle = "rgba(0,255,160,0.65)";
      ctx.lineWidth = 1.5 * dpr;
      ctx.stroke();
      ctx.restore();

      // Vertex dots with widx labels
      const dotR = Math.max(7 * dpr, W * 0.020);
      ctx.save();
      ctx.font = `bold ${Math.max(9 * dpr, W * 0.022)}px sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      pts.forEach((p, i) => {
        const r = (p.mm / maxThrow) * maxR;
        const a = ((p.deg - 90) * Math.PI) / 180;
        const x = cx + Math.cos(a) * r;
        const y = cy + Math.sin(a) * r;
        ctx.fillStyle = "rgba(255,160,40,0.95)";
        ctx.beginPath();
        ctx.arc(x, y, dotR, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "rgba(0,0,0,0.4)";
        ctx.lineWidth = 1 * dpr;
        ctx.stroke();
        ctx.fillStyle = "rgba(20,20,20,0.95)";
        const label = String((p.widx !== undefined ? p.widx : i) + 1);
        ctx.fillText(label, x, y);
      });
      ctx.restore();
    }

    // Central sprinkler marker: small filled circle with a soft ring.
    ctx.save();
    ctx.fillStyle = "rgba(150,180,210,0.85)";
    ctx.beginPath();
    ctx.arc(cx, cy, 3 * dpr, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(200,220,240,0.5)";
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    ctx.arc(cx, cy, 6 * dpr, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }

  _depthColor(ratio) {
    // rgba() string for fillStyle. Same hue mapping as the device's
    // zone_setup. Alpha kept high (0.65-0.85) so tiles read as solid
    // colors -- the post-tile blur softens edges but doesn't have
    // to fight low alpha to remain visible at small HA card sizes.
    let r, g, b, a;
    if (ratio < 0.6) {
      r = 40;
      g = 90 + Math.round((ratio / 0.6) * 80);
      b = 200;
      a = 0.65 + (ratio / 0.6) * 0.10;
    } else if (ratio < 0.88) {
      const t = (ratio - 0.6) / 0.28;
      r = 40;
      g = 170 + Math.round(t * 50);
      b = Math.round(200 * (1 - t));
      a = 0.75;
    } else if (ratio < 1.12) {
      const t = (ratio - 0.88) / 0.24;
      r = Math.round(40 + t * 120);
      g = 220;
      b = 0;
      a = 0.80;
    } else if (ratio < 1.5) {
      const t = (ratio - 1.12) / 0.38;
      r = 160 + Math.round(t * 80);
      g = Math.round(220 * (1 - t * 0.5));
      b = 0;
      a = 0.85;
    } else {
      r = 240;
      g = Math.max(0, 80 - Math.round((ratio - 1.5) * 40));
      b = 0;
      a = 0.90;
    }
    return `rgba(${r},${g},${b},${a.toFixed(2)})`;
  }

  _drawCenterText(ctx, cx, cy, text, dpr) {
    const k = dpr || 1;
    ctx.save();
    ctx.font = `${13 * k}px sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillStyle = "rgba(150,170,190,0.7)";
    ctx.fillText(text, cx, cy);
    ctx.restore();
  }
}

if (!customElements.get("irrigoto-heatmap-card")) {
  customElements.define("irrigoto-heatmap-card", IrrigotoHeatmapCard);
}

window.customCards = window.customCards || [];
window.customCards.push({
  type: "irrigoto-heatmap-card",
  name: "Irrigoto Heatmap",
  description: "Watering depth heatmap from an irrigoto device",
  preview: false,
});
