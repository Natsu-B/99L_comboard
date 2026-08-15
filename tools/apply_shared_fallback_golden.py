from pathlib import Path

path = Path("src/payload.rs")
text = path.read_text(encoding="utf-8")
old = """        let app = &frame.as_bytes()[LORA_PREFIX.len()..];
        assert_eq!(app.len(), 24);
        assert_eq!(&app[..4], &[0xa8, 0x01, 0x34, 0x01]);
        assert_eq!(app[23], xor_checksum(&app[..23]));
"""
new = """        assert_eq!(frame.as_bytes(), golden(\"LORA_MISSION_LINK_FALLBACK\"));
        let app = &frame.as_bytes()[LORA_PREFIX.len()..];
        assert_eq!(app.len(), 24);
        assert_eq!(&app[..4], &[0xa8, 0x01, 0x34, 0x01]);
        assert_eq!(app[23], xor_checksum(&app[..23]));
"""
if text.count(old) != 1:
    raise SystemExit("fallback golden assertion anchor was not unique")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
