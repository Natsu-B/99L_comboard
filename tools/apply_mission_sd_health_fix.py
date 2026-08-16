from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        print(f"{label}: already applied")
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: insertion point count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))
    print(f"{label}: applied")


protocol = Path("src/can/protocol.rs")
replace_once(
    protocol,
    "require_reserved_zero(id, 7, data[7], 0x78)?;",
    "require_reserved_zero(id, 7, data[7], 0x70)?;",
    "allow PowerTime Mission SD bit",
)
replace_once(
    protocol,
    "flags: 0x85,",
    "flags: 0x89,",
    "PowerTime golden flags",
)
replace_once(
    protocol,
    "CanRxMessage::decode_standard(CAN_ID_POWER_TIME, &[0, 0, 0, 0, 0, 0, 0, 0x08]),",
    "CanRxMessage::decode_standard(CAN_ID_POWER_TIME, &[0, 0, 0, 0, 0, 0, 0, 0x10]),",
    "PowerTime reserved-bit test",
)

lora = Path("src/tasks/lora_task_base.rs")
replace_once(
    lora,
    "status |= u32::from(power.flags & (1 << 2) != 0) << 10;",
    "status |= u32::from(power.flags & (1 << 3) != 0) << 10;",
    "map Mission SD health to A0 bit10",
)

golden = Path("testdata/99l_protocol_golden_vectors.txt")
replace_once(
    golden,
    "CAN_103=FCA0DCFAFF0C0085",
    "CAN_103=FCA0DCFAFF0C0089",
    "PowerTime golden vector",
)

print("ComBoard Mission SD health patch complete")
