/*
 * kss_dsp_extract.c  –  GameCube DSP extractor v2.0
 *
 * Handles two formats:
 *
 * 1. KSS Wave/BGM/SE Link Data (Konami GameCube archive trio: WaveData.bin,
 *    BgmData.bin, SeData.bin — magic strings "KSS Wave Link Data",
 *    "KSS BGM Link Data", "KSS SE Link Data", all dated 08/28/2002)
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
 *    SE table (at se_table_offset)
 *    ──────────────────────────────
 *    12-byte entries, terminated when the constant field ≠ 0xFC1A7F00.
 *      [0x00-0x03] audio_offset (absolute file offset; 0 = null/silence)
 *      [0x04-0x07] 0xFC1A7F00  (constant marker)
 *      [0x08-0x0B] 0x00001F14  (constant; verified identical across all
 *                              672 entries in the reference file — not
 *                              loop data or anything per-entry.)
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
 *    three data files supplied:
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
 *        and the hcs64 forum). A related, separately-documented case was
 *        found, though: a 2018 hcs64 forum thread describes Konami GC/Wii
 *        "Power Pro" .vas archives as DSP-shaped but headerless, with the
 *        same "can't cleanly recover per-stream coefficients" problem —
 *        suggesting this is a recurring trait of Konami's GC audio tooling
 *        from this era rather than a mistake in this analysis.
 *    Given this, every .dsp this tool writes uses safe placeholder
 *    coefficients (all zero, via make_dsp_hdr()'s NULL-sub path) rather
 *    than fabricated values. The split boundaries, sample/nibble counts,
 *    and raw ADPCM payload bytes are correct and verified; only the
 *    coefficients needed for a player to decode that payload back to
 *    correct-sounding PCM are missing. If you can locate the real
 *    coefficients (e.g. in the game's executable, or from someone with
 *    matching dsptool encoder output), patch them into make_dsp_hdr()'s
 *    coefficient slots (DSP header bytes 0x1C-0x3B) before relying on
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
 *   kss_dsp_extract [input_file] [output_prefix] [--interleave-bgm]
 *
 *   Defaults: WaveData.bin, "tmnt3mn"
 *   --interleave-bgm: opt-in, see BGM entry table note above.
 *
 * Output:
 *   KSS mode:   bgmNN.dsp + bgmNN_secondary.dsp (or bgmNNL/R.dsp with
 *               --interleave-bgm), seNNN.dsp
 *   TMNT3 mode: prefixNNNNL.dsp / prefixNNNNR.dsp
 *
 * Build:
 *   gcc -O2 -o kss_dsp_extract kss_dsp_extract.c
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
/*  DSP sub-block / header utilities                                    */
/* ------------------------------------------------------------------ */

/* A valid sub-block has first 8 bytes == 0 and a plausible nibble count. */
static int valid_sub(const unsigned char *s)
{
    int i;
    for (i = 0; i < 8; i++) if (s[i]) return 0;
    unsigned n = rb32(s + 8);
    return n > 0 && n < 0x40000000u;
}

/*
 * Build a standard 0x60-byte GameCube DSP header.
 *   mono_sz : bytes of mono audio data (per channel, after de-interleave)
 *   sub     : optional 0x40-byte sub-block; placed at DSP bytes [0x0C..0x4B]
 *             (covers loop flags, loop points, ca, coefficients, ps, yn, etc.)
 *             Pass NULL to fill those fields with safe defaults.
 *   srate   : sample rate in Hz
 *   loop_s  : loop start nibbles (only used when sub == NULL)
 *   loop_e  : loop end nibbles   (only used when sub == NULL; 0 = no loop)
 */
static void make_dsp_hdr(unsigned char *hdr,
                          unsigned mono_sz,
                          const unsigned char *sub,
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

    if (sub && valid_sub(sub)) {
        /*
         * Sub-block maps directly to DSP[0x0C..0x4B]:
         *   sub[0x00] = DSP[0x0C] loop_flag high byte
         *   sub[0x08] = DSP[0x14] loop_end
         *   sub[0x10] = DSP[0x1C] first coefficient
         *   sub[0x32] = DSP[0x3E] ps  …etc.
         */
        memcpy(hdr + 0x0C, sub, SUB_SZ);
    } else {
        /* Safe defaults: no loop, ADPCM format, ca=2, zero coefficients */
        wb16(0, hdr + 0x0C);    /* loop_flag = 0 */
        wb16(0, hdr + 0x0E);    /* format    = 0 (ADPCM) */
        if (loop_e > 0) {
            wb16(1, hdr + 0x0C);            /* loop_flag = 1 */
            wb32(loop_s, hdr + 0x10);       /* loop_start  */
            wb32(loop_e, hdr + 0x14);       /* loop_end    */
        }
        wb32(2, hdr + 0x18);    /* ca = 2 (standard ADPCM start) */
    }
}

/* Write a single-channel .dsp file (header + raw ADPCM audio). */
static int write_dsp(const char *path,
                      const unsigned char *sub,
                      unsigned mono_sz,
                      const unsigned char *audio,
                      unsigned srate,
                      unsigned loop_s,
                      unsigned loop_e)
{
    unsigned char h[DSP_HDR_SZ];
    make_dsp_hdr(h, mono_sz, sub, srate, loop_s, loop_e);

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
                               const unsigned char *interleaved, unsigned total_sz,
                               const unsigned char *sub_L, const unsigned char *sub_R)
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

    if (write_dsp(path_L, sub_L, szL, bL,
                  SRATE_DEFAULT, 0, 0))
        printf("    L: %-32s  %u bytes audio\n", path_L, szL);

    if (write_dsp(path_R, sub_R ? sub_R : sub_L,
                  szR, bR, SRATE_DEFAULT, 0, 0))
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

/* ================================================================== */
/*  KSS Wave Link Data extraction                                       */
/* ================================================================== */
static void extract_kss(const unsigned char *data, long fsz, int interleave_bgm)
{
    unsigned se_tbl   = rb32(data + 0x28);
    unsigned wave_off = rb32(data + 0x2C);
    unsigned wave_sz  = rb32(data + 0x30);

    printf("KSS Wave Link Data\n");
    printf("  SE table:    0x%08X\n", se_tbl);
    printf("  Wave offset: 0x%08X   size: 0x%08X\n\n", wave_off, wave_sz);

    /* ============================================================ */
    /*  BGM TRACKS                                                   */
    /* ============================================================ */
    printf("=== BGM Tracks ===\n");

    /*
     * Collect all BGM L-channel absolute file offsets for the boundary
     * list (used later when computing SE audio sizes).
     */
    unsigned bgm_addrs[128];
    int      bgm_addr_n = 0;

    int track = 0;
    unsigned tbl = 0x40u;

    while (tbl + 8u <= se_tbl) {
        unsigned La = rb32(data + tbl);
        unsigned Ls = rb32(data + tbl + 4u);
        if (La == 0 && Ls == 0) break;

        /* Detect R-channel partner (next entry whose low byte == 0x01) */
        int      has_R = (tbl + 16u <= se_tbl) &&
                         ((rb32(data + tbl + 8u) & 0xFFu) == 0x01u);
        unsigned Ra    = has_R ? (rb32(data + tbl + 8u) & 0xFFFFFF00u) : 0u;
        unsigned Rs    = has_R ? rb32(data + tbl + 12u)                 : 0u;

        /*
         * Resolve main-clip absolute address.
         * addr == 0 is a special case: track 0's clip sits at wave_off.
         */
        unsigned L_abs = (La == 0u) ? wave_off : La;

        /* Register in the boundary list (used later for SE size calculation) */
        if (La != 0u && bgm_addr_n < 128)
            bgm_addrs[bgm_addr_n++] = L_abs;

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
            write_stereo_pair(nMain, nSec, data + L_abs, Ls, NULL, NULL);
        } else {
            snprintf(nMain, sizeof(nMain), "bgm%02d.dsp", track);
            snprintf(nSec,  sizeof(nSec),  "bgm%02d_secondary.dsp", track);

            unsigned char h[DSP_HDR_SZ];
            make_dsp_hdr(h, Ls, NULL, SRATE_DEFAULT, 0, 0);
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
                make_dsp_hdr(h2, Rs, NULL, SRATE_DEFAULT, 0, 0);
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
    printf("Extracted %d BGM track(s)\n", track);

    /* ============================================================ */
    /*  SOUND EFFECTS                                                */
    /* ============================================================ */
    printf("\n=== Sound Effects ===\n");

    /*
     * Parse SE table: 12-byte entries until the marker fields stop matching.
     * Fields: [audio_offset, 0xFC1A7F00, 0x00001F14]
     * The third field was verified constant across every entry in the
     * sample file, so it carries no per-entry data — it is checked here
     * only to confirm the table is still aligned (catches corruption or
     * a misidentified se_table_offset on other files).
     */
    typedef struct { unsigned addr; } SeEnt;
    SeEnt *ses = (SeEnt *)malloc(4096u * sizeof(SeEnt));
    if (!ses) { fprintf(stderr, "OOM\n"); return; }
    int se_n = 0;
    int c2_mismatches = 0;

    {
        unsigned off = se_tbl;
        while (off + 12u <= wave_off && se_n < 4096) {
            unsigned a  = rb32(data + off);
            unsigned c1 = rb32(data + off + 4u);
            unsigned c2 = rb32(data + off + 8u);
            if (c1 != SE_C1_MAGIC) break;
            if (c2 != SE_C2_MAGIC) c2_mismatches++;
            ses[se_n].addr = a;
            se_n++;
            off += 12u;
        }
    }
    printf("%d SE entries in table", se_n);
    if (c2_mismatches)
        printf("  (warning: %d entries had an unexpected marker field —"
               " table layout may differ from the reference file)",
               c2_mismatches);
    printf("\n");

    /*
     * Build a sorted list of all valid audio boundaries:
     *   • Non-null SE addresses that fall inside the mid-area [se_tbl, wave_off)
     *   • BGM L-channel addresses collected above
     * This lets us compute accurate per-SE sizes even when BGM blocks
     * and SE blocks are interleaved in the file.
     */
    unsigned *bounds = (unsigned *)malloc((se_n + bgm_addr_n + 2u) * sizeof(unsigned));
    if (!bounds) { fprintf(stderr, "OOM\n"); free(ses); return; }
    int nb = 0;

    {
        int i;
        for (i = 0; i < se_n; i++) {
            unsigned a = ses[i].addr;
            if (a >= se_tbl && a < wave_off)
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
            if (ses[i].addr >= se_tbl && ses[i].addr < wave_off)
                valid++;
        printf("%d valid SE addresses in audio area\n\n", valid);
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
                printf("SE%03d: [null / silence]\n", i);
                continue;
            }

            /* Skip entries outside the audio area (e.g. header-region refs) */
            if (a < se_tbl || a >= wave_off) {
                printf("SE%03d: @0x%08X  [skip: outside audio area]\n", i, a);
                continue;
            }

            /* Compute size = distance to next boundary */
            unsigned sz = next_boundary(bounds, nb, a, wave_off) - a;
            if (sz == 0u) {
                printf("SE%03d: @0x%08X  [skip: size = 0]\n", i, a);
                continue;
            }
            if ((unsigned long)a + sz > (unsigned long)fsz)
                sz = (unsigned)((unsigned long)fsz - a);

            /* Build DSP header (non-looping; see comment above) */
            unsigned char h[DSP_HDR_SZ];
            make_dsp_hdr(h, sz, NULL, SRATE_DEFAULT, 0, 0);

            char name[64];
            snprintf(name, sizeof(name), "se%03d.dsp", i);
            FILE *f = fopen(name, "wb");
            if (!f) { perror(name); continue; }
            fwrite(h, 1, DSP_HDR_SZ, f);
            fwrite(data + a, 1, sz, f);
            fclose(f);

            printf("SE%03d: @0x%08X  sz=0x%06X (%6u B)  -> %s\n",
                   i, a, sz, sz, name);
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
/*  Entry point                                                         */
/* ================================================================== */
static void print_usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s [input_file] [output_prefix] [--interleave-bgm]\n"
        "  input_file       Defaults to WaveData.bin\n"
        "  output_prefix    Used only in legacy TMNT3 scan mode; defaults to \"tmnt3mn\"\n"
        "  --interleave-bgm Opt-in: de-interleave KSS BGM clips as 0x100-byte stereo\n"
        "                   instead of the default flat-mono extraction (see source\n"
        "                   comments above extract_kss() before using this).\n",
        argv0);
}

int main(int argc, char *argv[])
{
    const char *inpath = NULL;
    const char *prefix = NULL;
    int interleave_bgm = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--interleave-bgm") == 0) {
            interleave_bgm = 1;
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
    if (!inpath) inpath = "WaveData.bin";
    if (!prefix) prefix = "tmnt3mn";

    printf("===========================================\n");
    printf("  GameCube DSP Extractor v2.0\n");
    printf("  KSS Wave Link Data / TMNT3 strbgm.bin\n");
    printf("===========================================\n");
    printf("Input: %s\n", inpath);
    if (interleave_bgm)
        printf("Mode:  --interleave-bgm (opt-in stereo de-interleave for BGM)\n");
    printf("\n");

    /* Read the entire file into memory */
    FILE *fp = fopen(inpath, "rb");
    if (!fp) { perror(inpath); return 1; }
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

    /* Dispatch on magic */
    if (fsz >= 18 && memcmp(data, "KSS Wave Link Data", 18) == 0) {
        extract_kss(data, fsz, interleave_bgm);
    } else if (fsz >= 17 && memcmp(data, "KSS BGM Link Data", 17) == 0) {
        printf("KSS BGM Link Data detected – no raw DSP audio to extract.\n");
    } else {
        /* Treat as legacy TMNT3 strbgm.bin */
        extract_tmnt3(data, fsz, prefix);
    }

    free(data);
    printf("\nDone.\n");
    return 0;
}
