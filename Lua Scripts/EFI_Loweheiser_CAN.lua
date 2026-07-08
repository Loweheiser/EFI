--[[
  EFI Scripting backend driver for MegaSquirt-2 / MSExtra CAN Broadcast

  Target:
    - MegaSquirt-2 / MicroSquirt / MS2Extra CAN realtime broadcast
    - 11-bit standard CAN IDs
    - 500 kbit/s CAN bus, configured in ArduPilot params, not inside Lua
    - Publishes data into ArduPilot EFI scripting backend (EFI_TYPE = 7)

  Supported broadcast formats:
    MODE = 0: Simplified Dash Broadcast, default base ID 1512 decimal = 0x5E8
              Uses IDs: base+0 .. base+4
    MODE = 1: Advanced Real-Time Data Broadcast, default base ID 1520 decimal = 0x5F0
              Uses groups: 0, 1, 2, 3, 22 when available

  Required ArduPilot parameters, typical example:
    SCR_ENABLE      = 1
    EFI_TYPE        = 7
    CAN_P1_DRIVER   = 1
    CAN_D1_PROTOCOL = 10
    CAN_D1_BITRATE  = 500000
    EFI_MS_ENABLE   = 1
    EFI_MS_CANDRV   = 1

  MegaSquirt notes:
    - CAN broadcast uses 11-bit identifiers, sequential IDs, big-endian data.
    - Simplified Dash Broadcast automatic MS2 mode uses base ID 0x5E8 at 20 Hz.
    - Advanced RT default base ID is 0x5F0.

  This script is receive-only. It does not send throttle or commands to the ECU.
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: undefined-field
---@diagnostic disable: missing-parameter
---@diagnostic disable: need-check-nil

local SCRIPT_AP_VERSION = 4.3
local SCRIPT_NAME = "EFI: MegaSquirt CAN"
local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)
assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s requires %s %.1f, found %.1f', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))

local MAV_SEVERITY_ERROR = 3

-- Use a table key that should not conflict with the HFE script.
local PARAM_TABLE_KEY = 38
local PARAM_TABLE_PREFIX = "EFI_MS_"

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

local function get_time_sec()
    return millis():tofloat() * 0.001
end

-- Big-endian helpers, as specified by MegaSquirt CAN broadcast.
local function get_u8(frame, ofs)
    return frame:data(ofs)
end

local function get_u16_be(frame, ofs)
    return (frame:data(ofs) << 8) + frame:data(ofs + 1)
end

local function get_s16_be(frame, ofs)
    local v = get_u16_be(frame, ofs)
    if v >= 0x8000 then
        v = v - 0x10000
    end
    return v
end

local function f10_to_c(v)
    -- MegaSquirt temperatures are deg F * 10 in the broadcast docs.
    return ((v / 10.0) - 32.0) * (5.0 / 9.0)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 12), 'could not add EFI_MS param table')

--[[
// @Param: EFI_MS_ENABLE
// @DisplayName: Enable MegaSquirt CAN EFI driver
// @Description: Enable MegaSquirt CAN EFI Lua driver
// @Values: 0:Disabled,1:Enabled
// @User: Standard
--]]
local EFI_MS_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
// @Param: EFI_MS_RATE_HZ
// @DisplayName: MegaSquirt EFI update rate
// @Description: Lua script update rate. MS2 automatic dash broadcast is normally 20Hz.
// @Range: 1 200
// @User: Standard
--]]
local EFI_MS_RATE_HZ = bind_add_param('RATE_HZ', 2, 50)

--[[
// @Param: EFI_MS_CANDRV
// @DisplayName: MegaSquirt EFI CAN driver
// @Description: CAN driver to use for scripting CAN access
// @Values: 0:None,1:1stCANDriver,2:2ndCANDriver
// @User: Standard
--]]
local EFI_MS_CANDRV = bind_add_param('CANDRV', 3, 0)

--[[
// @Param: EFI_MS_MODE
// @DisplayName: MegaSquirt CAN broadcast mode
// @Description: Broadcast format to decode
// @Values: 0:DashBroadcast,1:AdvancedRealtimeBroadcast
// @User: Standard
--]]
local EFI_MS_MODE = bind_add_param('MODE', 4, 0)

--[[
// @Param: EFI_MS_BASE_ID
// @DisplayName: MegaSquirt base CAN ID
// @Description: Base CAN ID. Use 1512/0x5E8 for Dash Broadcast, 1520/0x5F0 for Advanced RT Broadcast. Decimal value.
// @Range: 0 2047
// @User: Standard
--]]
local EFI_MS_BASE_ID = bind_add_param('BASE_ID', 5, 1512)

--[[
// @Param: EFI_MS_BARO_KPA
// @DisplayName: Fallback barometric pressure
// @Description: Fallback atmospheric pressure in kPa. Used in Dash mode because Dash Broadcast does not include BARO.
// @Range: 50 120
// @User: Standard
--]]
local EFI_MS_BARO_KPA = bind_add_param('BARO_KPA', 6, 101.3)

--[[
// @Param: EFI_MS_FUEL_DTY
// @DisplayName: Fuel density
// @Description: Fuel density in grams per litre, reserved for future fuel flow estimation
// @Range: 0 2000
// @User: Standard
--]]
local EFI_MS_FUEL_DTY = bind_add_param('FUEL_DTY', 7, 740)

--[[
// @Param: EFI_MS_OPTIONS
// @DisplayName: MegaSquirt EFI options
// @Description: Bitmask options
// @Bitmask: 1:EnableCANLogging
// @User: Standard
--]]
local EFI_MS_OPTIONS = bind_add_param('OPTIONS', 8, 0)

local OPTION_LOGALLFRAMES = 0x01

if EFI_MS_ENABLE:get() == 0 then
    return
end

local CAN_BUF_LEN = 25
local driver1 = nil
if EFI_MS_CANDRV:get() == 1 then
    driver1 = CAN.get_device(CAN_BUF_LEN)
elseif EFI_MS_CANDRV:get() == 2 then
    driver1 = CAN.get_device2(CAN_BUF_LEN)
end

if not driver1 then
    gcs:send_text(0, string.format("%s: failed to load CAN driver", SCRIPT_NAME))
    return
end

local frame_count = 0
local function log_can_frame(frame)
    logger:write("CANF", 'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7', 'IBIBBBBBBBB',
        frame:id(), frame:dlc(), frame_count,
        frame:data(0), frame:data(1), frame:data(2), frame:data(3),
        frame:data(4), frame:data(5), frame:data(6), frame:data(7))
    frame_count = frame_count + 1
end

-- In ArduPilot 4.5.x and older, EFI temperatures were incorrectly interpreted as Celsius instead of Kelvin.
local temp_offset = 0.0
if FWVersion:major() == 4 and FWVersion:minor() <= 5 then
    temp_offset = -273.15
end

local efi_backend = nil
local now_s = get_time_sec()

local function engine_control(driver)
    local self = {}

    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    local C_TO_KELVIN = 273.15
    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()

    -- MegaSquirt values, decoded into engineering units.
    local rpm = 0
    local map_kpa = 0.0
    local baro_kpa = EFI_MS_BARO_KPA:get()
    local clt_c = 0.0
    local mat_c = 0.0
    local tps_pct = 0.0
    local batt_v = 0.0
    local pw1_ms = 0.0
    local pw2_ms = 0.0
    local adv_deg = 0.0
    local egt1_c = 0.0
    local afr1 = 0.0

    local function handle_dash_packet(frame, id)
        local base = EFI_MS_BASE_ID:get()
        local rel = id - base

        if rel == 0 then
            -- 0x5E8 default: MAP, RPM, CLT, TPS
            map_kpa = get_s16_be(frame, 0) / 10.0
            rpm = get_u16_be(frame, 2)
            clt_c = f10_to_c(get_s16_be(frame, 4))
            tps_pct = get_s16_be(frame, 6) / 10.0
            last_rpm_t = get_time_sec()
        elseif rel == 1 then
            -- 0x5E9 default: PW1, PW2, MAT, ADV
            pw1_ms = get_u16_be(frame, 0) / 1000.0
            pw2_ms = get_u16_be(frame, 2) / 1000.0
            mat_c = f10_to_c(get_s16_be(frame, 4))
            adv_deg = get_s16_be(frame, 6) / 10.0
        elseif rel == 2 then
            -- 0x5EA default: AFR target, AFR1, EGOcor1, EGT1, PWseq1
            afr1 = get_u8(frame, 1) / 10.0
            egt1_c = f10_to_c(get_s16_be(frame, 4))
            -- frame offset 6 has sequential pulsewidth if configured; leave pw1_ms from rel 1 as primary.
        elseif rel == 3 then
            -- 0x5EB default: Batt, sensors1, sensors2, knock retard, sensors3
            batt_v = get_s16_be(frame, 0) / 10.0
        end
    end

    local function handle_advanced_packet(frame, id)
        local base = EFI_MS_BASE_ID:get()
        local group = id - base

        if group == 0 then
            -- seconds, PW1, PW2, RPM
            pw1_ms = get_u16_be(frame, 2) / 1000.0
            pw2_ms = get_u16_be(frame, 4) / 1000.0
            rpm = get_u16_be(frame, 6)
            last_rpm_t = get_time_sec()
        elseif group == 1 then
            -- advance, status bits, AFR targets
            adv_deg = get_s16_be(frame, 0) / 10.0
        elseif group == 2 then
            -- BARO, MAP, MAT, CLT
            baro_kpa = get_s16_be(frame, 0) / 10.0
            map_kpa = get_s16_be(frame, 2) / 10.0
            mat_c = f10_to_c(get_s16_be(frame, 4))
            clt_c = f10_to_c(get_s16_be(frame, 6))
        elseif group == 3 then
            -- bytes[0]=squirt bitmask, bytes[1]=engine flags (NOT tps)
            -- bytes[2,3]=battV×10, bytes[4,5]=afr1×10, bytes[6,7]=afr2×10
            batt_v = get_s16_be(frame, 2) / 10.0
            afr1 = get_s16_be(frame, 4) / 10.0
        elseif group == 22 then
            -- EGT1..EGT4, deg F * 10
            egt1_c = f10_to_c(get_s16_be(frame, 0))
        end
    end

    local dbg_frames = 0
    local dbg_last_report = 0
    local dbg_backend_ok = false

    local dbg_raw = {}  -- stores last raw bytes per group id, for reporting

    local dbg_first = 0

    function self.handle_EFI_packet(frame)
        if frame:isExtended() then
            return
        end

        local dlc = frame:dlc()
        local id = frame:id():toint()
        local base = math.floor(EFI_MS_BASE_ID:get())
        local mode = math.floor(EFI_MS_MODE:get())

        if dbg_first < 3 then
            dbg_first = dbg_first + 1
            gcs:send_text(0, string.format("RAW: id=0x%X mode=%d base=%d dlc=%d",
                id, mode, base, dlc))
        end

        if mode == 0 then
            if dlc < 8 then return end
            if id >= base and id <= base + 4 then
                handle_dash_packet(frame, id)
            end
        else
            if dlc < 4 then return end
            local group = id - base
            if group == 0 or group == 2 or group == 3 then
                dbg_raw[group] = string.format("G%d dlc=%d: %02X %02X %02X %02X %02X %02X %02X %02X",
                    group, dlc,
                    frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                    frame:data(4), frame:data(5), frame:data(6), frame:data(7))
            end
            if group == 0 or group == 1 or group == 2 or group == 3 or group == 22 then
                handle_advanced_packet(frame, id)
            end
        end
    end

    function self.update_telemetry()
        local max_packets = 25
        local count = 0
        while count < max_packets do
            local frame = driver:read_frame()
            count = count + 1
            if not frame then
                break
            end
            dbg_frames = dbg_frames + 1
            if EFI_MS_OPTIONS:get() & OPTION_LOGALLFRAMES ~= 0 then
                log_can_frame(frame)
            end
            self.handle_EFI_packet(frame)
        end

        if last_rpm_t > last_state_update_t then
            last_state_update_t = last_rpm_t
            self.set_EFI_State()
        end

        -- Debug report every 5s: frames received and current decoded values.
        local now = get_time_sec()
        if now - dbg_last_report > 5.0 then
            dbg_last_report = now
            if not dbg_backend_ok then
                gcs:send_text(0, SCRIPT_NAME .. string.format(": frames=%d backend=%s",
                    dbg_frames, tostring(efi_backend ~= nil)))
            end
            gcs:send_text(0, string.format("EFI dbg: rpm=%d CLT=%.1fC MAT=%.1fC MAP=%.1f batt=%.1fV tps=%.1f",
                rpm, clt_c, mat_c, map_kpa, batt_v, tps_pct))
            -- dump raw bytes for layout diagnosis
            for _, s in pairs(dbg_raw) do
                gcs:send_text(0, s)
            end
        end
    end

    function self.set_EFI_State()
        cylinder_state:cylinder_head_temperature(clt_c + C_TO_KELVIN + temp_offset)
        cylinder_state:exhaust_gas_temperature(egt1_c + C_TO_KELVIN + temp_offset)
        cylinder_state:ignition_timing_deg(adv_deg)
        cylinder_state:injection_time_ms(pw1_ms)

        efi_state:engine_speed_rpm(uint32_t(rpm))
        efi_state:atmospheric_pressure_kpa(baro_kpa)
        efi_state:intake_manifold_pressure_kpa(map_kpa)
        efi_state:intake_manifold_temperature(mat_c + C_TO_KELVIN + temp_offset)
        efi_state:throttle_position_percent(math.max(0, math.min(100, math.floor(tps_pct + 0.5))))
        efi_state:ignition_voltage(batt_v)

        -- MS2 broadcast does not provide fuel pressure or fuel flow in the common dash groups.
        -- Leave them unset/zero rather than inventing values.
        efi_state:fuel_pressure_status(0)
        efi_state:fuel_consumption_rate_cm3pm(0)
        efi_state:estimated_consumed_fuel_volume_cm3(0)

        efi_state:cylinder_status(cylinder_state)
        efi_state:last_updated_ms(millis())
        efi_backend:handle_scripting(efi_state)
    end

    return self
end

local engine1 = engine_control(driver1)

local function update()
    now_s = get_time_sec()

    if not efi_backend then
        efi_backend = efi:get_backend(0)
        if not efi_backend then
            gcs:send_text(0, SCRIPT_NAME .. ": waiting for EFI backend...")
            return
        end
        gcs:send_text(0, SCRIPT_NAME .. ": EFI backend OK")
    end

    engine1.update_telemetry()
end

gcs:send_text(0, SCRIPT_NAME .. " loaded")

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / EFI_MS_RATE_HZ:get()
end

return protected_wrapper()
