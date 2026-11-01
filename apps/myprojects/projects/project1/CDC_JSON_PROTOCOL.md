# Project1 CDC JSON Protocol

## Transport

- Physical: USB CDC-ACM
- Current integration: send JSON request through NSH command `p1ctl`
- One request, one line response

Example:

```sh
p1ctl '{"id":1,"cmd":"sys.ping"}'
```

## Request Format

```json
{"id":1,"cmd":"param.set","key":"led_time","value":3}
```

- `id`: optional number, echoed back in response
- `cmd`: required command string
- other fields depend on command

## Response Format

Success:

```json
{"id":1,"ok":true,"code":0,"msg":"OK","data":{"pong":true}}
```

Error:

```json
{"id":1,"ok":false,"code":-22,"msg":"missing cmd"}
```

## Commands

- `sys.ping`
- `sys.info`
- `rfid.test` (optional `block`)
- `vtx.test` (optional `text`,`encoding`,`timeout_ms`)
- `param.list`
- `param.get` (`key`)
- `param.set` (`key`,`value`)
- `param.save`
- `param.reset`
- `wp.count`
- `wp.get` (`index`)
- `wp.set` (`index`,`hex`, optional `mode`,`led`,`announce`,`speed_en`,`speed`,`restore_ms`)
- `wp.clear` (optional `index`; omit index = clear ALL slots)
- `wp.save`
- `sys.reboot` (optional `to_dfu`, currently reserved)

## Parameter Keys

- `motor_speed_pct` (0~100)
- `rfid_target_block` (0~63)
- `led_time` (1~30, second)
- `match_threshold` (0~100, waypoint fuzzy-match similarity percent,
  default 60)

## Waypoint Commands (`wp.*`)

Up to 40 waypoints (index 0~39). Each waypoint matches the RFID card block
content (GBK bytes, max 16) and carries three actions.

Fields of `wp.set` / `wp.get`:

- `index`: 0~39
- `hex`: card content as hex string, spaces allowed
  (e.g. GBK "地球" = `"B5D8C7F2"`), max 16 bytes
- `mode`: 0 = fuzzy (Jaccard similarity vs `match_threshold`),
  1 = exact (byte-for-byte identical). Default 0.
- `led`: 1 = light functional LED on match. Default 0.
- `announce`: 1 = speak card content on match. Default 0.
- `speed_en`: 1 = override motor speed on match. Default 0.
- `speed`: target speed percent 0~100 (used when `speed_en`=1). Default 0.
- `restore_ms`: optional delayed recovery in milliseconds (0~60000).  When
  non-zero, the motor returns to the speed target active before this point
  after the requested delay.  Default 0 (keep the changed speed).

Notes:

- `wp.set` / `wp.clear` only modify RAM; call `wp.save` to persist
  (dedicated flash pages, independent from `param.save`).
- When a card matches no waypoint, the default behavior is LED + announce
  (backward compatible).
- The speed action fires on every successful card read; LED/announce keep
  the original dead-zone / LED-window gating.

Example: waypoint 0 = "地球", fuzzy match, LED only, slow down to 40%:

```sh
p1ctl '{"id":1,"cmd":"wp.set","index":0,"hex":"B5D8C7F2","mode":0,"led":1,"announce":0,"speed_en":1,"speed":40,"restore_ms":3000}'
p1ctl '{"id":2,"cmd":"wp.save"}'
```

## Typical Flow

1. `param.set` multiple times
2. `param.save` once
3. `sys.reboot`
4. `param.list` verify persistence after reboot

