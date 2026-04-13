# SpaceMouse 32-byte Packet Reverse Mapping (Hypothesis)

> ⚠️ NOTE:
> This mapping is inferred from empirical testing.
> Field positions are highly likely but NOT officially confirmed.

---

# 1. Packet Overview

Each received packet is **32 bytes (little-endian)**:

```

[0 ~ 3]   uint32  frame_id / counter / device state
[4 ~ 7]   uint32  button bitmask
[8 ~ 31]  int32[6] motion + status (6DoF + flags)

```

---

# 2. Byte Layout (Detailed)

## 2.1 Header

| Offset | Type   | Name       | Description |
|--------|--------|------------|-------------|
| 0–3    | uint32 | frame_id   | Incrementing counter or device frame index |

---

## 2.2 Button Mask

| Offset | Type   | Name     | Description |
|--------|--------|----------|-------------|
| 4–7    | uint32 | buttons  | Bitmask for mouse buttons |

### Button Bit Mapping (observed)

| Bit | Meaning        |
|-----|----------------|
| 0x01 | Left button    |
| 0x02 | Right button   |
| 0x04 | (unknown/mode) |
| 0x08 | (unknown state)|

---

## 2.3 Motion / 6DoF Block

All values are **signed int32 (little-endian)**.

| Offset | Axis        | Meaning |
|--------|-------------|---------|
| 8–11   | X linear    | Left/Right translation |
| 12–15  | Y linear    | Forward/Backward translation |
| 16–19  | Z linear    | Up/Down translation |
| 20–23  | X angular   | Roll (rotation around X) |
| 24–27  | Y angular   | Pitch (rotation around Y) |
| 28–31  | Z angular   | Yaw / status dependent |

---

# 3. Value Interpretation Rules

## 3.1 Endianness
All multi-byte integers are:

```

little-endian

```

Example:
```

0x0a000000 → 10
0xfbffffff → -5

```

---

## 3.2 Signed interpretation

All motion channels are:

```

int32 (signed)

```

Range:
```

-2,147,483,648 ~ 2,147,483,647

````

---

# 4. Observed Behavior Summary

## 4.1 Button behavior

- Left button → bit 0 (0x01)
- Right button → bit 1 (0x02)
- Multiple bits may be active simultaneously

---

## 4.2 Translation axes

### X axis
Changes clearly under:
- left/right movement

### Y axis
Changes under:
- forward/back movement

### Z axis
Changes under:
- up/down movement

---

## 4.3 Rotation axes (hypothesized)

- roll → offset 20–23
- pitch → offset 24–27
- yaw/status → offset 28–31

> NOTE: yaw may include mixed flags depending on device mode

---

# 5. Status / Unknown fields

## 5.1 frame_id (0–3)

Possible meanings:
- packet counter
- device timestamp frame
- or fixed zero-based header

---

## 5.2 last field ambiguity (28–31)

Observed behavior:
- sometimes smooth continuous motion (likely yaw)
- sometimes binary-like behavior (possible flags)

---

# 6. Python parsing reference (recommended)

```python
import struct

def parse_spacemouse_packet(data: bytes):
    assert len(data) == 32

    frame_id = struct.unpack_from("<I", data, 0)[0]
    buttons  = struct.unpack_from("<I", data, 4)[0]

    x = struct.unpack_from("<i", data, 8)[0]
    y = struct.unpack_from("<i", data, 12)[0]
    z = struct.unpack_from("<i", data, 16)[0]

    roll  = struct.unpack_from("<i", data, 20)[0]
    pitch = struct.unpack_from("<i", data, 24)[0]
    yaw   = struct.unpack_from("<i", data, 28)[0]

    return {
        "frame_id": frame_id,
        "buttons": buttons,
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }
````

---

# 7. Button decoding helper

```python
def decode_buttons(b):
    return {
        "left":  bool(b & 0x01),
        "right": bool(b & 0x02),
        "mode":  bool(b & 0x04),
        "flag":  bool(b & 0x08),
    }
```

---

# 8. Confidence level

| Field       | Confidence |
| ----------- | ---------- |
| buttons     | HIGH       |
| x/y/z       | HIGH       |
| frame_id    | MEDIUM     |
| rotation    | MEDIUM     |
| last uint32 | LOW-MEDIUM |

---

# 9. Recommended next validation step

To fully confirm mapping:

1. lock device physically
2. test one axis at a time
3. rotate only (no translation)
4. record min/max per axis
