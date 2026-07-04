/*
 * kss_dsp_extract.c  –  GameCube DSP extractor v2.2
 *
 * Default behavior extracts SOUND EFFECTS from WaveData.bin. BGM music
 * extraction is available but opt-in (pass --bgm) — earlier revisions of
 * this tool extracted both by default, which was more than was wanted.
 *
 * A second mode (--whole-file) skips real extraction entirely and wraps
 * the ENTIRE input file as one big single .dsp instead — see the comment
 * above extract_whole_file() for what that's actually good for (raw
 * exploration/debugging, not a real audio track).
 *
 * Windows drag-and-drop is supported directly: dropping a file onto the
 * compiled .exe runs it with just that one filename as an argument, which
 * this tool detects and responds to with a small interactive menu (since
 * a drag-and-drop gesture can't also carry command-line flags), then
 * pauses for Enter before closing so the console window doesn't vanish
 * before you can read the output. Any other invocation shape (flags
 * present, multiple arguments, or none at all) skips the menu and pause
 * entirely and behaves exactly as a normal command-line tool, so existing
 * scripts/automation are unaffected.
 *
 * Handles two formats:
 *
 * 1. KSS Wave/BGM/SE Link Data — Disney Sports: Soccer (GameCube, 2002),
 *    Konami's in-house audio archive trio: WaveData.bin, BgmData.bin,
 *    SeData.bin (magic strings "KSS Wave Link Data", "KSS BGM Link Data",
 *    "KSS SE Link Data", all dated 08/28/2002 — matching the game's 2002
 *    release). NOT the TMNT3 strbgm.bin format described in mode 2 below;
 *    the two were initially conflated since this tool started as an
 *    upgrade of a TMNT3-specific extractor, but the byte layouts are
 *    unrelated beyond both using GameCube DSP-ADPCM as the underlying
 *    sample codec.
 *
 *    WaveData.bin layout
 *    ────────────────────
 *    0x0000  Magic "KSS Wave Link Data  MM/DD/YYYY\0\0" (32 bytes)
 *    0x0020  uint32 (unused)
 *    0x0024  uint32 file_size
 *    0x0028  uint32 se_table_offset    (e.g. 0x3800)
 *    0x002C  uint32 wave_data_offset   (e.g. 0x1675000)
 *    0x0030  uint32 wave_data_size
 *    0x0040  BGM entry table (8 bytes/entry, two consecutive zeros = end)
 *
 *    BGM entry table
 *    ───────────────
 *    Entries come in pairs: a main entry, then an optional secondary entry
 *    flagged by a 0x01 low byte on its address word.
 *      main entry:      addr (absolute file offset; 0 → use wave_data_offset),
 *                        size in bytes
 *      secondary entry: (addr | 0x01), size in bytes
 *
 *    Verified against every BGM clip in the reference WaveData.bin (all 14
 *    tracks, head and tail of each, every 0x100-byte boundary): both the
 *    main and secondary clips are flat, continuous mono ADPCM-shaped data
 *    (100% valid predictor/scale nibbles throughout — no drop-outs at any
 *    boundary that would indicate stereo interleaving). The secondary clip
 *    is NOT a right audio channel: two different tracks were found to point
 *    at the exact same secondary-clip address, which a unique per-track
 *    channel could not do. Earlier drafts of this tool assumed GameCube-
 *    style 0x100-byte stereo interleaving (matching the original TMNT3
 *    strbgm.bin format below) — that assumption did not hold up against
 *    this file's actual bytes, so the default path now extracts both clips
 *    as independent flat-mono .dsp files. An opt-in --interleave-bgm flag
 *    is kept for other KSS-format files that may genuinely interleave (see
 *    write_stereo_pair()), but it is NOT exercised by default.
 *
 *    SE table — located by content, NOT by the header field
 *    ─────────────────────────────────────────────────────
 *    12-byte entries: [audio_offset:4][0xFC1A7F00:4][0x00001F14:4], the
 *    last two fields constant across every entry (verified, not loop data
 *    or anything else per-entry).
 *
 *    *** CORRECTED FINDING: the header field at 0x28 (0x00003800 in the
 *    reference file) is NOT the table's start offset, despite an earlier
 *    revision of this tool assuming so (and naming it se_table_offset).
 *    The table's true, content-verified extent is 1026 entries (1015 with
 *    real non-null audio addresses) running from file offset 0x2768 to
 *    0x5780 — but 0x3800 lands on entry #354 of that table, roughly a
 *    third of the way in, not entry #0. Because entries #354 onward still
 *    match the expected 12-byte record shape perfectly, parsing forward
 *    from 0x3800 alone gave no signal that anything was missing: it just
 *    silently produced a table 354 entries short (350 of them real audio),
 *    a ~35% undercount of actual sound effects. This was only caught by
 *    scanning the raw bytes for the record's exact 8-byte marker
 *    independent of any header field, and noticing the match run actually
 *    starts much earlier. find_se_table() now does exactly that scan
 *    instead of trusting the header, and is what this tool actually uses.
 *    The header field's real meaning is unconfirmed — it is printed at
 *    runtime for reference but no longer used operationally.
 *
 *    Between the end of the BGM table (~0x118) and the true SE table start
 *    (0x2768) sits a separate, still only partly understood structure —
 *    roughly 350 entries of its own, also on a 12-byte stride, holding a
 *    monotonically increasing value per entry rather than a file offset.
 *    It does NOT contain additional audio (no entry in it resembles a
 *    valid file pointer into the wave area) and does NOT look like ADPCM
 *    coefficients either — real per-stream LPC coefficients vary based on
 *    each frame's signal content and would not be smoothly monotonic the
 *    way every value in this structure is. The more likely explanation is
 *    some kind of cumulative seek/timing table, but this is a hypothesis,
 *    not a verified conclusion — it is left unparsed and unused rather
 *    than guessed at further.
 *
 *    SE audio size = distance to next occupied boundary in the file.
 *
 *    *** ADPCM COEFFICIENTS: NOT FOUND — extracted audio is NOT bit-exact ***
 *    GameCube DSP-ADPCM decoding needs 8 coefficient pairs (16 int16) per
 *    stream. The predictor nibbles actually used in this file's audio span
 *    0-7 (confirmed by direct sampling, not just assumed), which rules out
 *    a small fixed table  like the 5-entry one used by PS-ADPCM/PS2 SPU2-
 *    ADPCM, even though the project's companion SdSeCall.h header (Konami,
 *    2002, "SE Sound Call Label ... For PlayStation2(EE/IOP)") shows this
 *    audio system originated on PS2. A real per-stream coefficient table
 *    is expected to exist somewhere, but it was not found in any of the
 *    three data files supplied, including the not-yet-decoded structure
 *    described just above (ruled out for the monotonicity reason given
 *    there) and SeData.bin's own internal table (see below):
 *      - No header sits before any audio block in WaveData.bin itself
 *        (checked exhaustively, not just spot-checked).
 *      - BgmData.bin ("KSS BGM Link Data") is almost entirely zero-filled;
 *        its only content is one 16-byte block at offset 0x800, too small
 *        and structurally wrong-shaped to be 14 tracks' worth of coefficients.
 *      - SeData.bin ("KSS SE Link Data") does have real structured table
 *        data from offset 0x800 onward, but it did not decompose into a
 *        clean, consistent per-asset record format in the time available —
 *        it may hold coefficients in some other arrangement, or may be
 *        unrelated call/priority/routing metadata (consistent with the
 *        "Link Data" naming, i.e. ID→asset linking rather than waveform
 *        parameters). It is left unparsed rather than guessed at.
 *      - No public documentation for this exact "KSS ... Link Data" format
 *        was found (searched general web, vgmstream's source/format list,
 *        and hcs64.com's tool list — fetched in full directly, including
 *        the page hosting the original TMNT3 extractor this tool started
 *        from; it has no Disney Sports or "KSS" entry). A closely related,
 *        independently-documented case turned up, though: a 2018 hcs64
 *        forum thread describes Konami's later GC/Wii "Power Pro" .vas
 *        archives the same way — DSP-shaped audio with no way to recover
 *        per-stream coefficients, the archive itself "headerless." That
 *        same pattern (real per-stream coefficients normally required by
 *        DSP-ADPCM, but genuinely absent from the shipped data files) shows
 *        up repeatedly across other publicly-discussed "headerless"
 *        Konami/PS2-engine ports too, which is why this tool now treats it
 *        as an expected property of this codebase's tooling rather than a
 *        bug in this analysis.
 *    Given all this, the most likely remaining explanation is that
 *    coefficients for this title are baked into the game's executable
 *    (shared/global across streams, or selected by an in-memory table this
 *    tool has no access to) rather than shipped in any of these three data
 *    files — consistent with the format saving the per-clip header space
 *    a normal .dsp file would spend on them. Confirming that would need
 *    the game's DOL/executable, which was not provided here.
 *
 *    *** PLAYBACK NOTE: an all-zero coefficient placeholder (the first
 *    version of this tool's approach — "honest" in the sense of not
 *    fabricating values, but untested against a real decoder) turned out
 *    to make every extracted file outright FAIL to open in foobar2000 /
 *    vgmstream ("unsupported format or corrupted file"), not just decode
 *    incorrectly. Checked directly against vgmstream's own validation
 *    source (src/meta/ngc_dsp_std.c): it hard-rejects any header whose 16
 *    coefficients are all zero, and separately hard-rejects any header
 *    whose initial_ps field doesn't exactly match the real first byte of
 *    the audio data. make_dsp_hdr() now fixes both: initial_ps is always
 *    copied from the real audio (this part is fully correct, not a guess),
 *    and coefficients use a fixed non-zero placeholder, COEF_PLACEHOLDER —
 *    8 pairs of (2048, 0), a stable Q11 first-order "predict = previous
 *    sample" predictor — instead of all zeros, purely so the file passes
 *    validation and plays. It is still a placeholder, not the game's real
 *    per-stream coefficients: files now open and play, but will have
 *    audible quantization noise/distortion beyond what the original
 *    encoder intended, since the stream's scale values were chosen by the
 *    encoder assuming the real (unknown) predictor, not this generic one.
 *    If you locate the real coefficients (e.g. by dumping the game
 *    executable and searching it, or from someone with matching dsptool
 *    encoder output), replace COEF_PLACEHOLDER's values before relying on
 *    these files for anything beyond re-splitting/re-bundling raw data.
 *
 * 2. Legacy TMNT3 scan mode (original strbgm.bin from TMNT3 "Mutant Nightmare")
 *    Scans at 0x800-aligned offsets for entries whose second uint32 == 32000.
 *    Reconstructs L/R DSP headers from data at +0x80 / +0xC0 and de-interleaves.
 *    This mode's header reconstruction IS complete (coefficients come directly
 *    from the file, exactly as the original script already did) — only the
 *    KSS Wave Link Data mode above is missing coefficients.
 *
 * Usage:
 *   kss_dsp_extract [input_file] [output_prefix] [--bgm] [--interleave-bgm] [--whole-file]
 *
 *   Defaults: WaveData.bin, "tmnt3mn"
 *   --bgm:            opt-in, also extract BGM music (default is SE-only)
 *   --interleave-bgm: opt-in, only meaningful with --bgm; see BGM entry
 *                     table note above.
 *   --whole-file:     opt-in, wrap the whole input as one .dsp instead of
 *                     real extraction; see extract_whole_file() above.
 *
 *   Dropping a file onto the compiled .exe (exactly one argument, no
 *   flags) brings up an interactive menu to choose between normal
 *   extraction and --whole-file, since the drop gesture can't carry flags.
 *
 * Output:
 *   KSS mode:   seNNNN.dsp (always); with --bgm, also bgmNN.dsp +
 *               bgmNN_secondary.dsp (or bgmNNL/R.dsp with --interleave-bgm)
 *   --whole-file: <inputstem>_full.dsp (one file, whole input as its audio)
 *   TMNT3 mode: prefixNNNNL.dsp / prefixNNNNR.dsp
 *
 * Build:
 *   gcc -O2 -o kss_dsp_extract kss_dsp_extract.c
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                           */
/* ------------------------------------------------------------------ */
#define INTERLEAVE   0x100u   /* stereo block size (bytes)             */
#define SUB_SZ       0x40u    /* DSP sub-block size (bytes)            */
#define DSP_HDR_SZ   0x60u    /* standard GameCube DSP header size     */
#define SRATE_DEFAULT 32000u  /* sample rate used for all tracks       */
#define SE_C1_MAGIC  0xFC1A7F00u  /* constant in every SE table entry  */
#define SE_C2_MAGIC  0x00001F14u  /* second constant; verified fixed across
                                      all entries — used only to sanity-check
                                      table alignment, carries no per-entry data */

/* ------------------------------------------------------------------ */
/*  Big-endian helpers                                                  */
/* ------------------------------------------------------------------ */
static unsigned rb32(const unsigned char *p)
{
    return ((unsigned)p[0]<<24)|((unsigned)p[1]<<16)|
           ((unsigned)p[2]<<8) |(unsigned)p[3];
}
static void wb32(unsigned v, unsigned char *p)
{
    p[0]=(unsigned char)(v>>24); p[1]=(unsigned char)(v>>16);
    p[2]=(unsigned char)(v>>8);  p[3]=(unsigned char)(v);
}
static void wb16(unsigned v, unsigned char *p)
{
    p[0]=(unsigned char)(v>>8); p[1]=(unsigned char)(v);
}

/* ------------------------------------------------------------------ */
/*  DSP header construction                                             */
/* ------------------------------------------------------------------ */

/*
 * Build a standard 0x60-byte GameCube DSP header.
 *
 *   mono_sz : bytes of mono ADPCM audio data that will follow the header
 *   audio   : pointer to that audio data (must be readable for at least
 *             1 byte when mono_sz > 0) — needed to set initial_ps correctly,
 *             see note below
 *   srate   : sample rate in Hz
 *   loop_s  : loop start nibbles (0 if not looping)
 *   loop_e  : loop end nibbles   (0 = not looping)
 *
 * *** Two fields here matter more than they might look, and getting either
 * wrong makes real decoders (vgmstream, and anything built on it, including
 * foobar2000's vgmstream plugin) outright REJECT the file as "unsupported
 * format" rather than just mis-decoding it. Confirmed directly against
 * vgmstream's own validation source (src/meta/ngc_dsp_std.c,
 * read_dsp_header_endian()), not inferred:
 *
 * 1. initial_ps (header offset 0x3E) must equal the actual first byte of
 *    the audio data that follows. That byte packs the first ADPCM frame's
 *    predictor index (high nibble) and scale exponent (low nibble) — the
 *    header field is a redundant, must-match copy of it, and vgmstream
 *    hard-fails (`goto fail`) on any mismatch:
 *        if (header.initial_ps != read_u8(start_offset,sf)) goto fail;
 *    This is real, already-known-correct data (it's a byte that's
 *    genuinely present in the audio stream being written), not a guess —
 *    so this part of the header is fully accurate regardless of the
 *    coefficient situation below.
 *
 * 2. Coefficients (offset 0x1C, 16 × int16) must not be all zero.
 *    vgmstream counts zero entries and hard-fails if all 16 are zero:
 *        if (zero_coefs == 16) goto fail;
 *    An earlier revision of this tool used all-zero coefficients as an
 *    "honest placeholder" for the real per-stream values this format
 *    doesn't ship (see the top-of-file note on the missing coefficient
 *    search) — which is exactly what triggered this rejection on every
 *    extracted file.
 *
 *    *** COEF_PLACEHOLDER's specific values, and how they were chosen ***
 *    Getting past the all-zero check is necessary but not sufficient for
 *    listenable audio. A first attempt used a single (2048,0) pair (Q11
 *    "predict = previous sample") repeated across all 8 predictor slots.
 *    That passes validation but throws away real information: each ADPCM
 *    frame's header nibble selects one of 8 slots the ORIGINAL encoder
 *    picked adaptively per frame to fit that frame's signal — collapsing
 *    all 8 slots to identical behavior ignores that choice entirely.
 *    Measured on all 1011 extracted SE files, that uniform pair produced
 *    7.75% of decoded samples clipping at full scale — audible harsh
 *    distortion, consistent with reports of "still glitching" after the
 *    initial_ps/all-zero-coefficient fix above.
 *
 *    Using 8 *distinct* stable pairs (so each slot at least behaves
 *    differently, even with guessed values) dropped that to 0.99%. Trying
 *    to improve further by directly minimizing clipping rate via automated
 *    coordinate-descent search backfired: it converged toward near-zero
 *    coefficients, hitting 0.064% clipping — but mean output amplitude
 *    dropped from ~17800 (uniform) to ~3800, a >4x drop. That's a
 *    degenerate solution, not a better one: near-zero coefficients trivially
 *    avoid clipping by barely predicting anything, but the encoder chose
 *    each frame's scale/delta values assuming the decoder WOULD contribute
 *    meaningful predicted energy from history. Weak coefficients don't
 *    reconstruct the signal better, they just fail quietly instead of
 *    loudly. This is worth stating plainly since it's an easy trap: for
 *    this kind of problem, "minimize clipping" alone is not a valid proxy
 *    for "closer to correct."
 *
 *    The values below come from re-running that search with an added
 *    constraint — reject any candidate whose mean output amplitude drops
 *    below the 8-distinct-pair table's own amplitude — which found a real
 *    (if modest) improvement: 0.86% clipping across all 1011 files without
 *    sacrificing signal energy. All 8 pairs were checked for 2nd-order IIR
 *    stability (poles inside the unit circle); slot 7 has a thin stability
 *    margin (~0.018) that's fine for short SE clips (empirically verified
 *    across all 1011) but UNVERIFIED for long sustained BGM audio, where
 *    many more consecutive frames give marginal coefficients more room to
 *    drift — if you extract BGM (--bgm) and hear a slow building buzz/
 *    resonance rather than the more typical broadband quantization noise,
 *    that slot is the first thing to soften.
 *
 *    None of this makes the audio correct. It is still a synthetic
 *    placeholder chosen to minimize (not eliminate) the gap between
 *    "plays" and "sounds reasonable," not the game's real per-stream
 *    coefficients. Replace these values below if real coefficients are
 *    ever found (e.g. dumped from the game executable).
 */
static const int16_t COEF_PLACEHOLDER[16] = {
      -96,     0,
     1024,     0,
     1338,  -580,
     1792,  -512,
     1202,  -628,
     2868, -1408,
     3200, -1800,
     3500, -1488,
};

static void make_dsp_hdr(unsigned char *hdr,
                          unsigned mono_sz,
                          const unsigned char *audio,
                          unsigned srate,
                          unsigned loop_s,
                          unsigned loop_e)
{
    memset(hdr, 0, DSP_HDR_SZ);

    unsigned nib = mono_sz * 2u;
    unsigned smp = (unsigned)(((unsigned long long)nib * 7u) / 8u);

    wb32(smp,   hdr + 0x00);   /* num_samples  */
    wb32(nib,   hdr + 0x04);   /* num_nibbles  */
    wb32(srate, hdr + 0x08);   /* sample_rate  */

    wb16(0, hdr + 0x0C);    /* loop_flag = 0 (set below if looping) */
    wb16(0, hdr + 0x0E);    /* format    = 0 (ADPCM; vgmstream requires this exact value) */
    if (loop_e > 0) {
        wb16(1, hdr + 0x0C);            /* loop_flag = 1 */
        wb32(loop_s, hdr + 0x10);       /* loop_start (nibbles) */
        wb32(loop_e, hdr + 0x14);       /* loop_end   (nibbles) */
    }
    wb32(2, hdr + 0x18);    /* ca = 2 (standard ADPCM start address) */

    {
        int i;
        for (i = 0; i < 16; i++)
            wb16((unsigned)(uint16_t)COEF_PLACEHOLDER[i], hdr + 0x1C + i * 2);
    }
    wb16(0, hdr + 0x3C);    /* gain = 0 (vgmstream requires this exact value) */

    /* initial_ps: must equal the real first audio byte -- see note above. */
    if (mono_sz > 0 && audio != NULL)
        wb16(audio[0], hdr + 0x3E);
}

/* Write a single-channel .dsp file (header + raw ADPCM audio). */
static int write_dsp(const char *path,
                      unsigned mono_sz,
                      const unsigned char *audio,
                      unsigned srate,
                      unsigned loop_s,
                      unsigned loop_e)
{
    unsigned char h[DSP_HDR_SZ];
    make_dsp_hdr(h, mono_sz, audio, srate, loop_s, loop_e);

    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return 0; }
    fwrite(h, 1, DSP_HDR_SZ, f);
    fwrite(audio, 1, mono_sz, f);
    fclose(f);
    return 1;
}

/*
 * Split an interleaved L+R block into two mono .dsp files.
 * Blocks alternate every INTERLEAVE bytes: L₀ R₀ L₁ R₁ …
 * The last block may be shorter than INTERLEAVE.
 *
 * NOT called by extract_kss() by default: every BGM clip in the verified
 * reference file (WaveData.bin) is flat, continuous mono with no stereo
 * interleaving (see the detailed note above extract_kss()'s BGM loop).
 * This function is kept available — and exercised by the self-test below —
 * for KSS-format files from other titles that may genuinely interleave
 * stereo audio this way; pass --interleave-bgm on the command line to use
 * it instead of the flat-mono path.
 */
static void write_stereo_pair(const char *path_L, const char *path_R,
                               const unsigned char *interleaved, unsigned total_sz)
{
    /*
     * Compute exact L and R byte totals first. total_sz does not have to be
     * an even multiple of 2*INTERLEAVE — the final block can be a short,
     * single-channel remainder — so do not assume a 50/50 split when sizing
     * the buffers (that previously caused a heap overflow on real data).
     */
    unsigned capL = 0, capR = 0, pos = 0, blk = 0;
    while (pos < total_sz) {
        unsigned chunk = total_sz - pos;
        if (chunk > INTERLEAVE) chunk = INTERLEAVE;
        if (blk & 1u) capR += chunk; else capL += chunk;
        pos += chunk;
        blk++;
    }

    unsigned char *bL = (unsigned char *)calloc(1, capL ? capL : 1u);
    unsigned char *bR = (unsigned char *)calloc(1, capR ? capR : 1u);
    if (!bL || !bR) {
        fprintf(stderr, "Out of memory\n");
        free(bL); free(bR);
        return;
    }

    unsigned szL = 0, szR = 0;
    pos = 0; blk = 0;
    while (pos < total_sz) {
        unsigned chunk = total_sz - pos;
        if (chunk > INTERLEAVE) chunk = INTERLEAVE;
        if (blk & 1u) { memcpy(bR + szR, interleaved + pos, chunk); szR += chunk; }
        else           { memcpy(bL + szL, interleaved + pos, chunk); szL += chunk; }
        pos += chunk;
        blk++;
    }

    if (write_dsp(path_L, szL, bL, SRATE_DEFAULT, 0, 0))
        printf("    L: %-32s  %u bytes audio\n", path_L, szL);

    if (write_dsp(path_R, szR, bR, SRATE_DEFAULT, 0, 0))
        printf("    R: %-32s  %u bytes audio\n", path_R, szR);

    free(bL);
    free(bR);
}

/* ------------------------------------------------------------------ */
/*  Sorted boundary list (used for SE size calculation)                 */
/* ------------------------------------------------------------------ */

static int cmp_uint(const void *a, const void *b)
{
    unsigned ua = *(const unsigned *)a;
    unsigned ub = *(const unsigned *)b;
    return (ua > ub) - (ua < ub);
}

/* Given a sorted array, return the next value strictly greater than key. */
static unsigned next_boundary(const unsigned *arr, int n, unsigned key, unsigned fallback)
{
    int lo = 0, hi = n;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (arr[mid] <= key) lo = mid + 1;
        else hi = mid;
    }
    return lo < n ? arr[lo] : fallback;
}

/*
 * Locate the true SE table by content, not by trusting the file header.
 *
 * The header field at offset 0x28 was originally assumed to be the SE
 * table's start offset (it was even named se_table_offset in an earlier
 * revision of this tool). Verification against the reference WaveData.bin
 * disproved that: the table's real, content-verified extent is 1026
 * consecutive 12-byte entries, but the header field points to entry #354
 * of that table — roughly a third of the way in — not entry #0. Trusting
 * it silently dropped 354 entries (350 of them real, non-null audio),
 * a ~35% undercount of actual sound effects.
 *
 * This function instead scans for the exact 8-byte per-entry marker
 * (SE_C1_MAGIC immediately followed by SE_C2_MAGIC) at every byte
 * position in [search_start, search_end), takes the first match as the
 * table's first entry, then walks forward in fixed 12-byte steps for as
 * long as that same 8-byte marker keeps matching. Because the marker is
 * 8 essentially-arbitrary bytes, this run continuing correctly for over
 * a thousand consecutive 12-byte-strided entries is not a plausible
 * coincidence — it is a reliable signature of the real table, independent
 * of any (possibly mislabeled) header field.
 *
 * Returns 1 and sets *out_start / *out_count on success, 0 if no marker
 * occurrence was found at all in the search range.
 */
static int find_se_table(const unsigned char *data, long fsz,
                          unsigned search_start, unsigned search_end,
                          unsigned *out_start, int *out_count)
{
    unsigned char marker[8];
    wb32(SE_C1_MAGIC, marker);
    wb32(SE_C2_MAGIC, marker + 4);

    if (search_end > (unsigned)fsz) search_end = (unsigned)fsz;
    if (search_start + 8u > search_end) return 0;

    unsigned pos = search_start;
    unsigned first_match = 0;
    int found = 0;
    while (pos + 8u <= search_end) {
        if (memcmp(data + pos, marker, 8) == 0) {
            first_match = pos;
            found = 1;
            break;
        }
        pos++;
    }
    if (!found) return 0;

    unsigned table_start = first_match - 4u;  /* addr field precedes the marker */

    int count = 0;
    unsigned off = table_start;
    while (off + 12u <= (unsigned)fsz && memcmp(data + off + 4u, marker, 8) == 0) {
        count++;
        off += 12u;
    }

    *out_start = table_start;
    *out_count = count;
    return 1;
}

/* ================================================================== */
/*  KSS Wave Link Data extraction                                       */
/* ================================================================== */
static void extract_kss(const unsigned char *data, long fsz,
                         int interleave_bgm, int write_bgm)
{
    unsigned se_tbl_hdr = rb32(data + 0x28);
    unsigned wave_off = rb32(data + 0x2C);
    unsigned wave_sz  = rb32(data + 0x30);

    printf("KSS Wave Link Data\n");
    printf("  Header field @0x28: 0x%08X  (NOT the SE table start -- see below)\n", se_tbl_hdr);
    printf("  Wave offset: 0x%08X   size: 0x%08X\n\n", wave_off, wave_sz);

    /* ============================================================ */
    /*  BGM TABLE                                                    */
    /*  Always parsed (clip addresses are needed for the SE          */
    /*  boundary list below regardless), but .dsp files are only     */
    /*  written when write_bgm is set -- default is SE-only          */
    /*  extraction. Pass --bgm to also get BGM clips.                */
    /* ============================================================ */
    if (write_bgm) printf("=== BGM Tracks ===\n");

    /*
     * Collect all BGM L-channel absolute file offsets for the boundary
     * list (used later when computing SE audio sizes).
     */
    unsigned bgm_addrs[128];
    int      bgm_addr_n = 0;

    int track = 0;
    unsigned tbl = 0x40u;

    while (tbl + 8u <= wave_off) {
        unsigned La = rb32(data + tbl);
        unsigned Ls = rb32(data + tbl + 4u);
        if (La == 0 && Ls == 0) break;

        /* Detect R-channel partner (next entry whose low byte == 0x01) */
        int      has_R = (tbl + 16u <= wave_off) &&
                         ((rb32(data + tbl + 8u) & 0xFFu) == 0x01u);
        unsigned Ra    = has_R ? (rb32(data + tbl + 8u) & 0xFFFFFF00u) : 0u;
        unsigned Rs    = has_R ? rb32(data + tbl + 12u)                 : 0u;

        /*
         * Resolve main-clip absolute address.
         * addr == 0 is a special case: track 0's clip sits at wave_off.
         */
        unsigned L_abs = (La == 0u) ? wave_off : La;

        /* Register in the boundary list (used later for SE size calculation).
         * This always happens, even when write_bgm is off, since SE size
         * computation still needs to know where BGM clips sit in the file. */
        if (La != 0u && bgm_addr_n < 128)
            bgm_addrs[bgm_addr_n++] = L_abs;

        if (!write_bgm) {
            /* SE-only mode: skip writing/printing, just move to the next entry. */
            tbl += has_R ? 16u : 8u;
            track++;
            continue;
        }

        printf("BGM %02d: main@0x%08X  size=0x%08X", track, L_abs, Ls);
        if (has_R) printf("  secondary@0x%08X  size=0x%08X", Ra, Rs);
        printf("\n");

        /* Bounds check */
        if (Ls == 0u || L_abs >= (unsigned)fsz ||
            (unsigned long)L_abs + Ls > (unsigned long)fsz) {
            printf("  [skip: out of bounds or empty]\n");
            tbl += has_R ? 16u : 8u;
            track++;
            continue;
        }

        /*
         * NOTE on format: every BGM clip in the reference file was verified
         * byte-for-byte to be flat, continuous mono ADPCM — checked at every
         * 0x100-byte boundary across all 14 tracks (100% valid predictor/
         * scale nibbles throughout, head and tail). There is no embedded
         * DSP-style coefficient/loop header anywhere before these clips
         * (the preceding bytes are ordinary audio or silence, not a zeroed
         * header), and the table's second entry per track points to a
         * second, independently-sized flat-mono clip elsewhere in the file
         * rather than to an interleaved right channel — two tracks were
         * even confirmed to share the exact same secondary-clip address.
         * Earlier revisions of this tool assumed GameCube-style stereo
         * interleaving and a per-clip sub-header; neither held up against
         * the actual data, so both clips are now written as flat mono .dsp
         * files with default (non-looping) headers, exactly like the SE
         * extractor below. If your file's secondary clip is genuinely a
         * second audio channel meant to play in sync with the main clip,
         * it is still extracted correctly as its own standalone .dsp here
         * — it simply is not merged or treated as stereo automatically.
         */
        char nMain[64], nSec[64];

        if (interleave_bgm) {
            /*
             * Opt-in path (--interleave-bgm): treat the main clip as
             * 0x100-byte interleaved stereo and de-interleave it, ignoring
             * the secondary-clip entry. Use this only if you have separately
             * confirmed your file's BGM audio is actually interleaved —
             * it is NOT how WaveData.bin's BGM clips were verified to be
             * laid out (see note above).
             */
            snprintf(nMain, sizeof(nMain), "bgm%02dL.dsp", track);
            snprintf(nSec,  sizeof(nSec),  "bgm%02dR.dsp", track);
            write_stereo_pair(nMain, nSec, data + L_abs, Ls);
        } else {
            snprintf(nMain, sizeof(nMain), "bgm%02d.dsp", track);
            snprintf(nSec,  sizeof(nSec),  "bgm%02d_secondary.dsp", track);

            unsigned char h[DSP_HDR_SZ];
            make_dsp_hdr(h, Ls, data + L_abs, SRATE_DEFAULT, 0, 0);
            FILE *f = fopen(nMain, "wb");
            if (f) {
                fwrite(h, 1, DSP_HDR_SZ, f);
                fwrite(data + L_abs, 1, Ls, f);
                fclose(f);
                printf("    main:      %-32s  %u bytes audio\n", nMain, Ls);
            } else {
                perror(nMain);
            }

            if (has_R && Ra != 0u && Rs != 0u &&
                (unsigned long)Ra + Rs <= (unsigned long)fsz) {
                unsigned char h2[DSP_HDR_SZ];
                make_dsp_hdr(h2, Rs, data + Ra, SRATE_DEFAULT, 0, 0);
                FILE *f2 = fopen(nSec, "wb");
                if (f2) {
                    fwrite(h2, 1, DSP_HDR_SZ, f2);
                    fwrite(data + Ra, 1, Rs, f2);
                    fclose(f2);
                    printf("    secondary: %-32s  %u bytes audio\n", nSec, Rs);
                } else {
                    perror(nSec);
                }
            }
        }

        tbl += has_R ? 16u : 8u;
        track++;
    }
    if (write_bgm) printf("Extracted %d BGM track(s)\n\n", track);

    /* ============================================================ */
    /*  SOUND EFFECTS  (default / primary extraction target)        */
    /* ============================================================ */
    printf("=== Sound Effects ===\n");

    /*
     * The file header's field at offset 0x28 does NOT mark the true SE
     * table start. An earlier revision of this tool assumed it did (and
     * named it se_table_offset) because starting a 12-byte-stride parse
     * there happened to find 672 entries that all matched the expected
     * [addr][0xFC1A7F00][0x00001F14] shape — a false positive caused by
     * that field actually pointing to entry #354 of the table's true
     * 1026 entries, i.e. partway in rather than at the start; every entry
     * from there onward still matches the same repeating record shape,
     * so nothing about that scan looked wrong from entry #354 onward.
     * The 354 entries before it (350 with real, non-null audio) were
     * silently missed as a result. This was only caught by scanning the
     * file for the record's exact 8-byte marker independent of the
     * header, and noticing the match run actually starts much earlier.
     * The header field's true meaning is unconfirmed; it is printed
     * above for reference but not used operationally here.
     */
    unsigned se_tbl;
    int se_n = 0;
    if (!find_se_table(data, fsz, 0x118u, wave_off, &se_tbl, &se_n)) {
        fprintf(stderr, "Could not locate the SE table by content scan -- "
                         "this file's format may differ from the reference "
                         "WaveData.bin. No SE files extracted.\n");
        return;
    }
    printf("SE table found at 0x%08X: %d entries (located by scanning for the "
           "record's exact byte signature, not by trusting the header)\n",
           se_tbl, se_n);

    typedef struct { unsigned addr; } SeEnt;
    SeEnt *ses = (SeEnt *)malloc((size_t)se_n * sizeof(SeEnt));
    if (!ses) { fprintf(stderr, "OOM\n"); return; }
    {
        unsigned off = se_tbl;
        int i;
        for (i = 0; i < se_n; i++) {
            ses[i].addr = rb32(data + off);
            off += 12u;
        }
    }
    unsigned se_tbl_end = se_tbl + (unsigned)se_n * 12u;

    /*
     * Build a sorted list of all valid audio boundaries:
     *   • Non-null SE addresses that fall inside the mid-area [se_tbl_end, wave_off)
     *   • BGM main-clip addresses collected above
     * This lets us compute accurate per-SE sizes even when BGM blocks
     * and SE blocks are interleaved in the file.
     */
    unsigned *bounds = (unsigned *)malloc(((unsigned)se_n + (unsigned)bgm_addr_n + 2u) * sizeof(unsigned));
    if (!bounds) { fprintf(stderr, "OOM\n"); free(ses); return; }
    int nb = 0;

    {
        int i;
        for (i = 0; i < se_n; i++) {
            unsigned a = ses[i].addr;
            if (a >= se_tbl_end && a < wave_off)
                bounds[nb++] = a;
        }
        for (i = 0; i < bgm_addr_n; i++)
            bounds[nb++] = bgm_addrs[i];
        bounds[nb++] = wave_off;    /* upper sentinel */
    }
    qsort(bounds, (size_t)nb, sizeof(unsigned), cmp_uint);

    /* Deduplicate */
    {
        int i, k = 0;
        for (i = 0; i < nb; i++)
            if (i == 0 || bounds[i] != bounds[i-1])
                bounds[k++] = bounds[i];
        nb = k;
    }

    /* Count valid SE entries */
    {
        int i, valid = 0;
        for (i = 0; i < se_n; i++)
            if (ses[i].addr >= se_tbl_end && ses[i].addr < wave_off)
                valid++;
        printf("%d valid (non-null, in-range) SE addresses\n\n", valid);
    }

    /*
     * Extract each SE in table order. No loop-point source exists in this
     * table, so every SE .dsp is written non-looping (loop_flag = 0).
     */
    int written = 0;
    {
        int i;
        for (i = 0; i < se_n; i++) {
            unsigned a = ses[i].addr;

            /* Skip null entries */
            if (a == 0u) {
                continue;
            }

            /* Skip entries outside the audio area (e.g. header-region refs) */
            if (a < se_tbl_end || a >= wave_off) {
                printf("SE%04d: @0x%08X  [skip: outside audio area]\n", i, a);
                continue;
            }

            /* Compute size = distance to next boundary */
            unsigned sz = next_boundary(bounds, nb, a, wave_off) - a;
            if (sz == 0u) {
                printf("SE%04d: @0x%08X  [skip: size = 0]\n", i, a);
                continue;
            }
            if ((unsigned long)a + sz > (unsigned long)fsz)
                sz = (unsigned)((unsigned long)fsz - a);

            /* Build DSP header (non-looping; see comment above) */
            unsigned char h[DSP_HDR_SZ];
            make_dsp_hdr(h, sz, data + a, SRATE_DEFAULT, 0, 0);

            char name[64];
            snprintf(name, sizeof(name), "se%04d.dsp", i);
            FILE *f = fopen(name, "wb");
            if (!f) { perror(name); continue; }
            fwrite(h, 1, DSP_HDR_SZ, f);
            fwrite(data + a, 1, sz, f);
            fclose(f);

            written++;
        }
    }
    printf("\nWrote %d SE file(s)\n", written);

    free(ses);
    free(bounds);
}

/* ================================================================== */
/*  Legacy TMNT3 scan mode  (original strbgm.bin algorithm)            */
/* ================================================================== */
static void extract_tmnt3(const unsigned char *data, long fsz,
                           const char *prefix)
{
    const unsigned HEADSIZE   = 0x100u;
    const unsigned INTERLEAVE_LEGACY = 0x100u;
    const unsigned ALIGN      = 0x800u;

    unsigned search = 0u;
    unsigned count  = 0u;

    printf("Legacy TMNT3 scan mode (prefix: %s)\n\n", prefix);

    while ((long)(search + 8u) <= fsz) {
        unsigned dsp_sz = rb32(data + search);
        unsigned srate  = rb32(data + search + 4u);

        if (srate != 32000u) {
            search += ALIGN;
            continue;
        }
        if ((unsigned long)search + dsp_sz > (unsigned long)fsz) {
            search += ALIGN;
            continue;
        }

        printf("%04u:0x%08X  size=0x%08X  freq=%u\n",
               count, search, dsp_sz, srate);

        unsigned audio_sz  = dsp_sz - HEADSIZE;   /* total interleaved bytes */
        unsigned chan_nib  = audio_sz;             /* nibbles per channel     */
        unsigned chan_smp  = audio_sz * 14u / 8u / 2u;

        /* ---- Build L header ---- */
        unsigned char hL[DSP_HDR_SZ];
        memset(hL, 0, DSP_HDR_SZ);
        wb32(chan_smp, hL + 0x00);
        wb32(chan_nib, hL + 0x04);
        wb32(srate,   hL + 0x08);

        /* Copy sub-block from +0x80 (covers DSP[0x0C..0x4B]) */
        if (search + 0x80u + SUB_SZ <= (unsigned)fsz)
            memcpy(hL + 0x0C, data + search + 0x80u, SUB_SZ);

        /* ---- Build R header ---- */
        unsigned char hR[DSP_HDR_SZ];
        memset(hR, 0, DSP_HDR_SZ);
        wb32(chan_smp, hR + 0x00);
        wb32(chan_nib, hR + 0x04);
        wb32(srate,   hR + 0x08);

        if (search + 0xC0u + SUB_SZ <= (unsigned)fsz)
            memcpy(hR + 0x0C, data + search + 0xC0u, SUB_SZ);

        /* ---- Write L file ---- */
        char nL[256], nR[256];
        snprintf(nL, sizeof(nL), "%s%04uL.dsp", prefix, count);
        snprintf(nR, sizeof(nR), "%s%04uR.dsp", prefix, count);

        FILE *fL = fopen(nL, "wb");
        FILE *fR = fopen(nR, "wb");
        if (!fL || !fR) {
            if (fL) fclose(fL);
            if (fR) fclose(fR);
            perror("open");
            break;
        }

        fwrite(hL, 1, DSP_HDR_SZ, fL);
        fwrite(hR, 1, DSP_HDR_SZ, fR);

        /* De-interleave audio at 0x100-byte blocks */
        unsigned pos = search + HEADSIZE;
        unsigned j;
        for (j = 0u; j < audio_sz && (long)(pos + INTERLEAVE_LEGACY * 2u) <= fsz;
             j += INTERLEAVE_LEGACY * 2u) {
            fwrite(data + pos,                    1, INTERLEAVE_LEGACY, fL);
            fwrite(data + pos + INTERLEAVE_LEGACY, 1, INTERLEAVE_LEGACY, fR);
            pos += INTERLEAVE_LEGACY * 2u;
        }

        fclose(fL);
        fclose(fR);
        printf("  -> %s  %s\n", nL, nR);

        count++;
        search = ((search + dsp_sz + 0xFFFu) / ALIGN) * ALIGN;
    }

    printf("\nExtracted %u DSP pair(s)\n", count);
}

/* ================================================================== */
/*  Whole-file mode: wrap the ENTIRE input file as a single .dsp        */
/* ================================================================== */
/*
 * This does not parse the KSS format at all -- it treats every byte of
 * the input, from offset 0 to EOF, as one continuous mono ADPCM stream
 * and slaps a single 0x60-byte header on the front.
 *
 * Be clear about what this is actually useful for: it is NOT a way to
 * get one clean audio file out of WaveData.bin. Most of the file is
 * headers, tables, and 1000+ independently-authored clips, each of
 * which resets its own ADPCM predictor history at its own frame 0 --
 * concatenating all of that and decoding it as if it were one take
 * will produce a jarring, glitchy jumble even before accounting for
 * the still-unsolved missing-coefficients problem. What it IS useful
 * for: a quick way to eyeball/ear-ball the raw byte layout of a file
 * you haven't reverse engineered yet (silence gaps between sections
 * are often audible even when the "audio" in between isn't meaningful),
 * or to sanity-check that a file loads at all before writing a real
 * parser for it.
 */
static void extract_whole_file(const unsigned char *data, long fsz, const char *inpath)
{
    printf("=== Whole-file mode ===\n");
    printf("Wrapping the entire %ld-byte input as one mono .dsp.\n", fsz);
    printf("NOTE: this is a raw diagnostic dump, not a real audio track --\n");
    printf("      most of it will not decode as coherent sound. See the\n");
    printf("      comment above extract_whole_file() in the source for why.\n\n");

    if ((unsigned long)fsz > 0x10000000ul) {
        fprintf(stderr,
            "Warning: input is larger than 256MB. num_nibbles (file_size*2) would\n"
            "exceed vgmstream's validation limit (0x20000000) and the resulting\n"
            ".dsp would fail to open. Proceeding anyway, but expect a rejection.\n");
    }

    /* Derive "<inputstem>_full.dsp" from the input path, stripping any
     * directory components and the original extension. */
    const char *base = inpath;
    const char *slash1 = strrchr(inpath, '/');
    const char *slash2 = strrchr(inpath, '\\');
    if (slash1 && (!slash2 || slash1 > slash2)) base = slash1 + 1;
    else if (slash2) base = slash2 + 1;

    char stem[240];
    snprintf(stem, sizeof(stem), "%s", base);
    char *dot = strrchr(stem, '.');
    if (dot) *dot = '\0';

    char outname[256];
    snprintf(outname, sizeof(outname), "%s_full.dsp", stem);

    unsigned char h[DSP_HDR_SZ];
    make_dsp_hdr(h, (unsigned)fsz, data, SRATE_DEFAULT, 0, 0);

    FILE *f = fopen(outname, "wb");
    if (!f) { perror(outname); return; }
    fwrite(h, 1, DSP_HDR_SZ, f);
    fwrite(data, 1, (size_t)fsz, f);
    fclose(f);

    printf("Wrote %s  (%u byte header + %ld bytes audio)\n", outname, DSP_HDR_SZ, fsz);
}

/* ================================================================== */
/*  Entry point                                                         */
/* ================================================================== */
static void print_usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [input_file] [output_prefix] [--bgm] [--interleave-bgm] [--whole-file]\n"
        "  input_file       Defaults to WaveData.bin\n"
        "  output_prefix    Used only in legacy TMNT3 scan mode; defaults to \"tmnt3mn\"\n"
        "  --bgm            Opt-in: also extract BGM music clips (default is\n"
        "                   sound-effects-only extraction).\n"
        "  --interleave-bgm Opt-in, only meaningful together with --bgm: de-interleave\n"
        "                   KSS BGM clips as 0x100-byte stereo instead of the default\n"
        "                   flat-mono extraction (see source comments above\n"
        "                   extract_kss() before using this).\n"
        "  --whole-file     Skip normal extraction; wrap the ENTIRE input file as one\n"
        "                   big single .dsp instead (raw diagnostic dump -- see the\n"
        "                   comment above extract_whole_file() before relying on this).\n"
        "\n"
        "Drag-and-drop: dropping a file onto the .exe with no other options runs this\n"
        "with just that one argument, which brings up an interactive menu to choose\n"
        "between normal extraction and whole-file mode, since a dropped file can't\n"
        "carry command-line flags with it.\n",
        argv0);
}

int main(int argc, char *argv[])
{
    const char *inpath = NULL;
    const char *prefix = NULL;
    int interleave_bgm = 0;
    int write_bgm = 0;
    int whole_file = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--interleave-bgm") == 0) {
            interleave_bgm = 1;
        } else if (strcmp(argv[i], "--bgm") == 0) {
            write_bgm = 1;
        } else if (strcmp(argv[i], "--whole-file") == 0) {
            whole_file = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (!inpath) {
            inpath = argv[i];
        } else if (!prefix) {
            prefix = argv[i];
        } else {
            fprintf(stderr, "Unexpected argument: %s\n\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    /*
     * Drag-and-drop detection: Windows Explorer launches
     * "kss_dsp_extract.exe <dropped_file>" with exactly one argument and
     * no way to attach flags to the gesture. That's indistinguishable from
     * someone typing just the filename at a prompt, which is fine -- in
     * both cases there's no way to know which mode they want, so ask.
     * Any other invocation shape (flags present, multiple positional args,
     * or no arguments at all) is assumed to be deliberate/scripted use and
     * runs straight through with no prompts, so automation keeps working.
     */
    int interactive = (argc == 2 && argv[1][0] != '-');

    if (!inpath) inpath = "WaveData.bin";
    if (!prefix) prefix = "tmnt3mn";

    printf("===========================================\n");
    printf("  GameCube DSP Extractor v2.2\n");
    printf("  Disney Sports: Soccer 'KSS ... Link Data' / TMNT3 strbgm.bin\n");
    printf("===========================================\n");
    printf("Input: %s\n", inpath);

    if (interactive) {
        printf("\nNo command-line options given -- choose a mode:\n");
        printf("  1) Extract sound effects as separate .dsp files [default]\n");
        printf("  2) Convert the WHOLE file to a single .dsp (raw diagnostic dump;\n");
        printf("     most of it will NOT sound like coherent audio)\n");
        printf("Enter choice [1]: ");
        fflush(stdout);
        char line[16];
        if (fgets(line, sizeof(line), stdin) && line[0] == '2')
            whole_file = 1;
        printf("\n");
    }

    printf("Mode:  %s\n",
           whole_file ? "whole-file (single raw .dsp dump)"
                      : (write_bgm ? "SE + BGM" : "SE only (default; pass --bgm for music too)"));
    if (!whole_file && interleave_bgm)
        printf("       --interleave-bgm\n");
    printf("\n");

    /* Read the entire file into memory */
    FILE *fp = fopen(inpath, "rb");
    if (!fp) {
        perror(inpath);
        if (interactive) {
            printf("\nPress Enter to exit...");
            fflush(stdout);
            int c; while ((c = getchar()) != '\n' && c != EOF) {}
        }
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    unsigned char *data = (unsigned char *)malloc((size_t)fsz);
    if (!data) { fprintf(stderr, "Out of memory\n"); fclose(fp); return 1; }
    if (fread(data, 1, (size_t)fsz, fp) != (size_t)fsz) {
        fprintf(stderr, "Read error\n"); fclose(fp); free(data); return 1;
    }
    fclose(fp);
    printf("File size: 0x%08lX (%ld bytes)\n\n", (unsigned long)fsz, fsz);

    if (whole_file) {
        extract_whole_file(data, fsz, inpath);
    } else if (fsz >= 18 && memcmp(data, "KSS Wave Link Data", 18) == 0) {
        extract_kss(data, fsz, interleave_bgm, write_bgm);
    } else if (fsz >= 17 && memcmp(data, "KSS BGM Link Data", 17) == 0) {
        printf("KSS BGM Link Data detected – no raw DSP audio to extract.\n");
    } else {
        /* Treat as legacy TMNT3 strbgm.bin */
        extract_tmnt3(data, fsz, prefix);
    }

    free(data);
    printf("\nDone.\n");

    if (interactive) {
        printf("\nPress Enter to exit...");
        fflush(stdout);
        int c; while ((c = getchar()) != '\n' && c != EOF) {}
    }
    return 0;
}
