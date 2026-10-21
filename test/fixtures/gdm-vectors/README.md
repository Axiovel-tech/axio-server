# `.gdm` / `.gdmb` golden vectors (vendored)

Copied byte for byte from AxioStack `contracts/gdm-vectors/` at commit
`62c1601` (branch `docs/gdm-format-spec`, AxioStack PR #84), which also holds
the normative spec `contracts/gdm-format.md` and the generator `gdm_ref.py`.

- **v1 conformance** (spec §10): `flat-single/` and `edge/`: 21 reject vectors
  (20 files and the oversize recipe in `edge/index.json`) and 2 accept
  vectors.
- **Provisional** (spec §4.11, not part of conformance): `provisional/`
  exercises the MANEUVER, RTH_PLAN and ENVELOPES layouts that a v1 reader
  refuses.

Only the files the tests read are copied: each vector's `show.gdm`, raw
`d01.gdmb` and `expected.json`, and the edge `.gdmb.hex` files with their
`index.json`.

Do not edit these files. When the spec changes, regenerate the vectors in
AxioStack and copy them again. To test against an AxioStack checkout
directly, point `GDM_VECTORS_DIR` at its `contracts/gdm-vectors` directory:

```bash
GDM_VECTORS_DIR=../../contracts/gdm-vectors uv run pytest test/test_show_gdmb.py
```
