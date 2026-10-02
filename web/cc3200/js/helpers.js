function be32(v) {
    return new Uint8Array([
        (v >>> 24) & 0xFF,
        (v >>> 16) & 0xFF,
        (v >>> 8) & 0xFF,
        v & 0xFF
    ]);
}

function le16From(u8, off) {
    return (u8[off] | (u8[off + 1] << 8)) >>> 0;
}

function le32From(u8, off) {
    return (u8[off] | (u8[off + 1] << 8) | (u8[off + 2] << 16) | (u8[off + 3] << 24)) >>> 0;
}

function be32From(u8, off) {
    return (((u8[off] << 24) >>> 0) | (u8[off + 1] << 16) | (u8[off + 2] << 8) | u8[off + 3]) >>> 0;
}

function u8Concat(parts) {
    let total = 0;
    for (const p of parts) total += p.length;
    const out = new Uint8Array(total);
    let off = 0;
    for (const p of parts) {
        out.set(p, off);
        off += p.length;
    }
    return out;
}

function parseIntAuto(s) {
    if (typeof s !== 'string') return NaN;
    const t = s.trim();
    if (!t) return NaN;
    return parseInt(t, t.startsWith('0x') || t.startsWith('0X') ? 16 : 10);
}

