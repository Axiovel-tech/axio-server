# `.gdm` / `.gdmb` golden vectors (vendored)

Copied byte for byte from AxioStack `contracts/gdm-vectors/` at commit
`81a81ff` (branch `docs/gdm-format-spec`, AxioStack PR #84), which also holds
the normative spec `contracts/gdm-format.md` and the generator `gdm_ref.py`.
Only the files the tests read are copied: each vector's `show.gdm`, raw
`d01.gdmb` and `expected.json`, and every `edge/*.gdmb.hex` with
`edge/index.json`.

Do not edit these files. When the spec changes, regenerate the vectors in
AxioStack and copy them again. To test against an AxioStack checkout
directly, point `GDM_VECTORS_DIR` at its `contracts/gdm-vectors` directory:

```bash
GDM_VECTORS_DIR=../../contracts/gdm-vectors uv run pytest test/test_show_gdmb.py
```
