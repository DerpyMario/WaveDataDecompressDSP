/*
 * wavedata0_extract.c - Konami "KSS Wave" WaveData0.bin extractor (v3.0)
 *
 * Target: Disney Sports: Soccer (GameCube, Konami 2002) - WaveData0.bin
 *         Konami KSS sound driver ("Control Code Call Label for GameCube
 *         Version 1.00, based on PlayStation2 Sound Simulator v2.30").
 *
 * ============================================================================
 *  WHY EARLIER EXTRACTIONS SOUNDED LIKE NOISE
 * ============================================================================
 *  The previous tool (kss_dsp_extract) concluded that this format ships no
 *  ADPCM coefficients and substituted a hand-tuned placeholder table. That
 *  conclusion was wrong. WaveData0.bin contains a full per-sample header
 *  table with real GameCube DSP-ADPCM coefficients at file offset 0x5740,
 *  0x40 bytes per sample, 1184 samples. Because the archive's structures are
 *  BYTE-PACKED (3-byte fields, so u32/s16 fields land on offsets that are
 *  not 4-byte aligned), a scan that assumed 4-byte-aligned records walked
 *  straight past them. Decoding DSP-ADPCM with wrong coefficients does not
 *  fail loudly - it reconstructs a plausible-looking waveform with the wrong
 *  predictor, which is exactly the "high noise distortion" that was heard.
 *
 *  Three further bugs compounded it:
 *    - the sound-entry table was parsed on a grid shifted by 7 bytes, so
 *      every sample offset was the *previous* entry's offset;
 *    - offsets were read as u32 (value<<8) instead of the packed u24 they
 *      are, inflating them far past end-of-file;
 *    - sizes were guessed as "distance to the next non-zero boundary"
 *      instead of read from the nibble-count field, so clips ran into the
 *      following sample.
 *
 * ============================================================================
 *  VERIFIED FORMAT OF WaveData0.bin  (all offsets are physical, big-endian)
 * ============================================================================
 *  Note: this file has no magic string. Structures are packed, not aligned.
 *
 *  0x0000  Group/bank table, 16 bytes per entry, TRUNCATED AT FILE START.
 *          Only the last 6 of the 13 entries survive in this file. Each
 *          surviving entry ends with u24 group_size<<8 at entry+12. Those
 *          six values (0x22A800, 0x193D60, 0x1FB200, 0x20B3C0, 0x2A4680,
 *          0x204C40) match, byte for byte, the group sizes this tool
 *          computes from the sample tables for groups 7..12 - which is what
 *          confirms the group layout below is right.
 *
 *  0x0F84  Group directory, 13 entries x 8 bytes:
 *              +0  u8  sample_count for this group
 *              +4  u24 unknown<<8 (monotonic; not an offset into this file)
 *          Counts: 34,124,91,97,93,78,78,84,70,99,125,121,90 = 1184 total,
 *          which is exactly the number of sound entries and of sample
 *          headers found independently below.
 *
 *  0x1F3C  Sound entry table, 1184 entries x 12 bytes (ends 0x56BC exactly):
 *              +0  u24 offset of the sample within its group (32-byte units)
 *              +3  s16 fine tune (0, -998 or -1735; constant per group)
 *              +5  u8  volume (always 0x7F)
 *              +6  u8  pan    (always 0x00)
 *              +7  u16 0
 *              +9  u16 0x1F14 (constant across all 1184 entries)
 *              +11 u8  0
 *          The offset field restarts at 0 at each group boundary, and the
 *          restarts land exactly on the group counts from 0x0F84.
 *
 *  0x5740  Sample header table, 1184 entries x 0x40 bytes (ends 0x17F40):
 *              +0   u32 flags (0, or 0x200 = SD_BST_LOOP from SdCtrlCall.h)
 *              +4   u24 nibble count (DSP-ADPCM nibbles; bytes = nibbles/2)
 *              +7   u32 format (always 2 = DSP-ADPCM)
 *              +11  s16 x 16  ADPCM coefficients  <-- THE MISSING PIECE
 *              +43  u32 root key (0..120, MIDI-style base note)
 *              +47  17 bytes padding
 *
 *  0x17F43 Audio data. Sample data is plain mono GameCube DSP-ADPCM,
 *          8-byte frames, no per-clip header. Groups are laid out in order;
 *          group base = 0x17F43 + sum of preceding group sizes, and each
 *          group size = max(entry_offset + align32(bytes)) over its entries.
 *
 *  Alignment proof: decoding every frame of all 1184 samples at base 0x17F43
 *  yields 2,939,556 frames with 0 invalid predictor indices (>7) and 0
 *  invalid scale exponents (>12). Shifting the base by -3..+1 or to the next
 *  32-byte boundary yields ~45% invalid predictors. The base is not a guess.
 *
 *  Not present in this file: sample rate. 32000 Hz (the GameCube DSP norm)
 *  is used by default; --rate overrides it. Root key and fine tune are
 *  emitted in the manifest so a sampler can retune per entry. Mapping the
 *  SD_* IDs in SdSeCall.h (0x1000..0x1576) onto these 1184 samples needs
 *  SeData.bin, which was not supplied.
 *
 * ============================================================================
 *  Build:  gcc -O2 -o wavedata0_extract wavedata0_extract.c -lm
 *  Usage:  wavedata0_extract WaveData0.bin [-o outdir] [--wav] [--dsp]
 *                            [--rate 32000] [--group N] [--index N]
 *                            [--flat] [--stats]
 *          Default output is both .wav and .dsp, into ./extracted,
 *          organised as group00/se0000.wav ... (--flat for one directory).
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/stat.h>

#define NGROUPS      13
#define GROUP_DIR    0x0F84u
#define SE_TABLE     0x1F3Cu
#define HDR_TABLE    0x5740u
#define HDR_STRIDE   0x40u
#define SE_STRIDE    12u
#define AUDIO_BASE   0x17F43u
#define FRAME_BYTES  8
#define FRAME_SAMPS  14

static uint32_t rb16(const uint8_t *p){ return ((uint32_t)p[0]<<8)|p[1]; }
static uint32_t rb24(const uint8_t *p){ return ((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|p[2]; }
static uint32_t rb32(const uint8_t *p){ return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static int16_t  rs16(const uint8_t *p){ return (int16_t)rb16(p); }

static void wb16(uint8_t *p, uint32_t v){ p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static void wb32(uint8_t *p, uint32_t v){ p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static void wl16(uint8_t *p, uint32_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void wl32(uint8_t *p, uint32_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

typedef struct {
    uint32_t group, index_in_group, global_index;
    uint32_t offset;        /* absolute file offset of ADPCM data */
    uint32_t nibbles, bytes, samples;
    uint32_t flags, root_key, format;
    int32_t  tune;
    uint32_t vol, pan;
    int16_t  coef[16];
} Entry;

static int16_t clamp16(int32_t v){ return v > 32767 ? 32767 : (v < -32768 ? -32768 : (int16_t)v); }

/* Standard GameCube DSP-ADPCM decode. Returns samples written. */
static uint32_t dsp_decode(const uint8_t *src, uint32_t nbytes, uint32_t nsamples,
                           const int16_t *coef, int16_t *dst, uint32_t *clipped)
{
    int32_t hist1 = 0, hist2 = 0;
    uint32_t out = 0, i = 0;
    while (i < nbytes && out < nsamples) {
        uint8_t ps = src[i++];
        int32_t scale = 1 << (ps & 0x0F);
        int idx = (ps >> 4) & 0x0F;          /* real data never exceeds 7 */
        int32_t c1 = coef[(idx & 7) * 2], c2 = coef[(idx & 7) * 2 + 1];
        for (int n = 0; n < FRAME_SAMPS && out < nsamples; n++) {
            if (i + (n >> 1) >= nbytes) break;
            uint8_t byte = src[i + (n >> 1)];
            int32_t nib = (n & 1) ? (byte & 0x0F) : (byte >> 4);
            if (nib > 7) nib -= 16;                       /* sign extend */
            int32_t s = (nib * scale) << 11;
            s += 1024 + c1 * hist1 + c2 * hist2;
            s >>= 11;
            int16_t v = clamp16(s);
            if (clipped && (s > 32767 || s < -32768)) (*clipped)++;
            dst[out++] = v;
            hist2 = hist1; hist1 = v;
        }
        i += 7;
    }
    return out;
}

static uint32_t dsp_nibbles_to_samples(uint32_t nib)
{
    uint32_t frames = nib / 16, rem = nib % 16;
    uint32_t s = frames * FRAME_SAMPS;
    if (rem > 2) s += rem - 2;
    return s;
}

static void write_dsp(const char *path, const Entry *e, const uint8_t *audio, uint32_t rate)
{
    uint8_t h[0x60];
    memset(h, 0, sizeof h);
    wb32(h + 0x00, e->samples);
    wb32(h + 0x04, e->nibbles);
    wb32(h + 0x08, rate);
    wb16(h + 0x0C, (e->flags & 0x200) ? 1 : 0);          /* loop flag      */
    wb16(h + 0x0E, 0);                                    /* format: ADPCM  */
    wb32(h + 0x10, (e->flags & 0x200) ? 2 : 0);           /* loop start nib */
    wb32(h + 0x14, (e->flags & 0x200) ? e->nibbles - 1 : 0);
    wb32(h + 0x18, 2);                                    /* initial offset */
    for (int i = 0; i < 16; i++) wb16(h + 0x1C + 2 * i, (uint16_t)e->coef[i]);
    wb16(h + 0x3C, 0);                                    /* gain           */
    wb16(h + 0x3E, audio[0]);      /* initial_ps MUST equal first data byte */
    wb16(h + 0x40, 0);                                    /* hist1          */
    wb16(h + 0x42, 0);                                    /* hist2          */
    wb16(h + 0x44, audio[0]);                             /* loop_ps        */
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return; }
    fwrite(h, 1, sizeof h, f);
    fwrite(audio, 1, e->bytes, f);
    if (e->bytes & 0x1F) { uint8_t z[32] = {0}; fwrite(z, 1, 32 - (e->bytes & 0x1F), f); }
    fclose(f);
}

static void write_wav(const char *path, const int16_t *pcm, uint32_t n, uint32_t rate)
{
    uint8_t h[44];
    uint32_t data = n * 2;
    memcpy(h, "RIFF", 4);      wl32(h + 4, 36 + data);
    memcpy(h + 8, "WAVEfmt ", 8); wl32(h + 16, 16);
    wl16(h + 20, 1); wl16(h + 22, 1);
    wl32(h + 24, rate); wl32(h + 28, rate * 2);
    wl16(h + 32, 2); wl16(h + 34, 16);
    memcpy(h + 36, "data", 4); wl32(h + 40, data);
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "cannot write %s\n", path); return; }
    fwrite(h, 1, sizeof h, f);
    fwrite(pcm, 2, n, f);
    fclose(f);
}

static void mkdirp(const char *p)
{
#ifdef _WIN32
    mkdir(p);
#else
    mkdir(p, 0755);
#endif
}

/* ======================================================================== *
 *  SeData.bin  ->  SD_* symbol names for each wave
 *
 *  SeData.bin ("KSS SE Link Data  08/28/2002") layout, all big-endian:
 *    0x00  char[32] magic/date
 *    0x24  u32      file size
 *    0x28  { u32 count; u32 rel_offset; } x n, terminated by count == 0
 *             -> 256,256,256,256,256,119 = 1399 sound-effect IDs
 *                at 0x800 + rel_offset, 16 bytes each
 *    0x800 SE entry table, 16 bytes per entry:
 *             +0  u8  ?           (8 / 9 / 10)
 *             +1  u8  wave group   <- index of the WaveData0 bank in slot B
 *             +2  u8  bank id A    (0xFF = none)
 *             +3  u8  bank id B
 *             +4  u8  voice count  (1..4)
 *             +11 u8  flags
 *             +14 u16 offset of this SE's data block, relative to 0x800
 *    data block:
 *             u32 x voice_count   offsets of each voice stream, block-relative
 *             then the voice streams, each terminated by 0xFF
 *
 *  Voice stream: byte-oriented commands, 0xFF = end, values < 0x80 are notes
 *  with 3 operands.  Command 0xD3 / 0xD4 = program (wave) select, 2 operands:
 *  operand 0 is a level/pitch value, operand 1 is the program byte P:
 *      P & 0x80 == 0  ->  wave  P        of group 0   (always-resident bank)
 *      P & 0x80 != 0  ->  wave (P & 0x7F) of group entry[+1] (or 1 if that is 0)
 *  Verified: for every group the maximum decoded index is exactly the group's
 *  sample count minus one, with no out-of-range value anywhere.
 * ======================================================================== */

#define SE_MAXNAMES 8
typedef struct { char n[SE_MAXNAMES][40]; int cnt; } Names;

static int seq_args(unsigned op)
{
    if (op < 0x80) return 3;                 /* note: key + 3 operands      */
    switch (op) {
    case 0x97: case 0x9d: case 0xd0: case 0xd2: case 0xd8: case 0xd9:
    case 0xdd: case 0xde: case 0xdf: case 0xe0: case 0xe7: case 0xeb:
    case 0xf2:                       return 1;
    case 0xd3: case 0xd4: case 0xdc: case 0xe6: case 0xf0: case 0xf9:
    case 0xfb:                       return 2;
    case 0xe1: case 0xe4: case 0xf1: case 0xf6: case 0xfd: return 3;
    case 0x98:                       return 0;
    default:                         return -1;
    }
}

/* returns number of SEs that resolved to at least one wave, or -1 on error */
static long load_names(const char *sedata_path, const char *header_path,
                       const uint32_t *counts, uint32_t total, Names *out,
                       FILE *mapf)
{
    FILE *f = fopen(sedata_path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", sedata_path); return -1; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *s = malloc((size_t)sz);
    if (!s || fread(s, 1, (size_t)sz, f) != (size_t)sz) { fclose(f); return -1; }
    fclose(f);
    if (sz < 0x900 || memcmp(s, "KSS SE Link Data", 16)) {
        fprintf(stderr, "%s is not a KSS SE Link Data file\n", sedata_path);
        free(s); return -1;
    }

    uint32_t nse = 0;
    for (uint32_t b = 0; 0x28 + 8 * b + 8 <= 0x800; b++) {
        uint32_t c = rb32(s + 0x28 + 8 * b);
        if (!c) break;
        nse += c;
    }
    printf("SeData: %u sound-effect IDs\n", nse);

    /* symbol names from SdSeCall.h ------------------------------------- */
    char (*sym)[40] = calloc(nse, 40);
    if (header_path) {
        FILE *h = fopen(header_path, "rb");
        if (!h) fprintf(stderr, "cannot open %s, IDs will be numeric\n", header_path);
        else {
            char line[1024]; unsigned long got = 0;
            while (fgets(line, sizeof line, h)) {
                char nm[64]; unsigned v;
                if (sscanf(line, " #define %63s 0x%x", nm, &v) == 2 &&
                    v >= 0x1000 && v < 0x1000 + nse && !sym[v - 0x1000][0]) {
                    snprintf(sym[v - 0x1000], 40, "%s", nm); got++;
                }
            }
            fclose(h);
            printf("names: %lu symbols read from %s\n", got, header_path);
        }
    }

    uint32_t gbase[NGROUPS]; uint32_t acc = 0;
    for (int g = 0; g < NGROUPS; g++) { gbase[g] = acc; acc += counts[g]; }

    if (mapf) fprintf(mapf, "se_id,se_name,wave_group,voices,wave_global_indices\n");

    long resolved = 0;
    for (uint32_t i = 0; i < nse; i++) {
        const uint8_t *e = s + 0x800 + 16 * i;
        uint32_t grp   = e[1];
        uint32_t nv    = e[4];
        uint32_t blk   = 0x800 + rb16(e + 14);
        uint32_t slotB = grp ? grp : 1;
        if (grp >= NGROUPS || nv == 0 || nv > 8 || blk + 4 * nv > (uint32_t)sz) continue;

        uint32_t hit[64]; int nhit = 0;
        for (uint32_t v = 0; v < nv; v++) {
            uint32_t o = blk + rb32(s + blk + 4 * v);
            if (o >= (uint32_t)sz) continue;
            for (uint32_t p = o; p < (uint32_t)sz; ) {
                unsigned op = s[p];
                if (op == 0xFF) break;
                int na = seq_args(op);
                if (na < 0) break;                     /* unknown command   */
                if (op == 0xD3 || op == 0xD4) {
                    unsigned P = s[p + 2];
                    uint32_t g = (P & 0x80) ? slotB : 0;
                    uint32_t k = P & 0x7F;
                    if (k < counts[g] && nhit < 64) {
                        uint32_t gi = gbase[g] + k;
                        int dup = 0;
                        for (int q = 0; q < nhit; q++) if (hit[q] == gi) dup = 1;
                        if (!dup) hit[nhit++] = gi;
                    }
                }
                p += 1 + (uint32_t)na;
            }
        }
        const char *name = sym[i][0] ? sym[i] : NULL;
        char fallback[24];
        if (!name) { snprintf(fallback, sizeof fallback, "SE_%04X", 0x1000 + i); name = fallback; }
        if (nhit) resolved++;
        if (mapf) {
            fprintf(mapf, "0x%04X,%s,%u,%u,", 0x1000 + i, name, slotB, nv);
            for (int q = 0; q < nhit; q++) fprintf(mapf, "%u%s", hit[q], q + 1 == nhit ? "" : " ");
            fputc('\n', mapf);
        }
        for (int q = 0; q < nhit; q++) {
            Names *N = &out[hit[q]];
            if (N->cnt < SE_MAXNAMES) snprintf(N->n[N->cnt++], 40, "%s", name);
        }
    }
    printf("SeData: %ld of %u sound effects reference at least one wave\n", resolved, nse);
    uint32_t named = 0;
    for (uint32_t i = 0; i < total; i++) if (out[i].cnt) named++;
    printf("names: %u of %u waves identified\n", named, total);
    free(sym); free(s);
    return resolved;
}

int main(int argc, char **argv)
{
    const char *in = NULL, *outdir = "extracted";
    uint32_t rate = 32000;
    int want_wav = 0, want_dsp = 0, flat = 0, stats = 0, big = 0;
    long only_group = -1, only_index = -1;
    const char *sedata = NULL, *hdrfile = NULL;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--wav"))   want_wav = 1;
        else if (!strcmp(argv[i], "--dsp"))   want_dsp = 1;
        else if (!strcmp(argv[i], "--flat"))  flat = 1;
        else if (!strcmp(argv[i], "--stats")) stats = 1;
        else if (!strcmp(argv[i], "--big"))   big = 1;
        else if (!strcmp(argv[i], "--sedata") && i + 1 < argc) sedata  = argv[++i];
        else if (!strcmp(argv[i], "--names")  && i + 1 < argc) hdrfile = argv[++i];
        else if (!strcmp(argv[i], "-o")    && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "--rate")&& i + 1 < argc) rate = (uint32_t)strtoul(argv[++i], 0, 0);
        else if (!strcmp(argv[i], "--group") && i + 1 < argc) only_group = strtol(argv[++i], 0, 0);
        else if (!strcmp(argv[i], "--index") && i + 1 < argc) only_index = strtol(argv[++i], 0, 0);
        else if (argv[i][0] != '-') in = argv[i];
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 1; }
    }
    if (!in) in = "WaveData0.bin";
    if (!want_wav && !want_dsp) want_wav = want_dsp = 1;

    FILE *f = fopen(in, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", in); return 1; }
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *d = malloc((size_t)fsz);
    if (!d || fread(d, 1, (size_t)fsz, f) != (size_t)fsz) { fprintf(stderr, "read failed\n"); return 1; }
    fclose(f);
    if (fsz < (long)AUDIO_BASE) { fprintf(stderr, "file too small to be WaveData0.bin\n"); return 1; }

    /* ---- group directory ------------------------------------------------ */
    uint32_t counts[NGROUPS], total = 0;
    for (int g = 0; g < NGROUPS; g++) { counts[g] = d[GROUP_DIR + 8 * g]; total += counts[g]; }
    printf("groups: ");
    for (int g = 0; g < NGROUPS; g++) printf("%u%s", counts[g], g == NGROUPS - 1 ? "" : ",");
    printf("  total entries: %u\n", total);

    if (SE_TABLE + total * SE_STRIDE > (uint32_t)fsz ||
        HDR_TABLE + total * HDR_STRIDE > (uint32_t)fsz) {
        fprintf(stderr, "table extents exceed file - not the expected format\n"); return 1;
    }

    Entry *E = calloc(total, sizeof *E);
    for (uint32_t i = 0; i < total; i++) {
        const uint8_t *se = d + SE_TABLE  + SE_STRIDE  * i;
        const uint8_t *hd = d + HDR_TABLE + HDR_STRIDE * i;
        E[i].global_index = i;
        E[i].offset   = rb24(se);            /* group-relative for now */
        E[i].tune     = rs16(se + 3);
        E[i].vol      = se[5];
        E[i].pan      = se[6];
        E[i].flags    = rb32(hd);
        E[i].nibbles  = rb24(hd + 4);
        E[i].format   = rb32(hd + 7);
        for (int c = 0; c < 16; c++) E[i].coef[c] = rs16(hd + 11 + 2 * c);
        E[i].root_key = rb32(hd + 43);
        E[i].bytes    = E[i].nibbles / 2;
        E[i].samples  = dsp_nibbles_to_samples(E[i].nibbles);
        if (E[i].format != 2)
            fprintf(stderr, "warning: entry %u has format %u (expected 2)\n", i, E[i].format);
    }

    /* ---- group bases: groups are stored back to back ------------------- */
    uint32_t base = AUDIO_BASE, idx = 0;
    for (int g = 0; g < NGROUPS; g++) {
        uint32_t gsize = 0;
        for (uint32_t j = 0; j < counts[g]; j++) {
            Entry *e = &E[idx + j];
            uint32_t end = e->offset + ((e->bytes + 31) & ~31u);
            if (end > gsize) gsize = end;
            e->group = (uint32_t)g;
            e->index_in_group = j;
        }
        for (uint32_t j = 0; j < counts[g]; j++) E[idx + j].offset += base;
        base += gsize;
        idx  += counts[g];
    }
    if (base > (uint32_t)fsz)
        fprintf(stderr, "warning: computed audio end 0x%X exceeds file size 0x%lX\n", base, fsz);
    else
        printf("audio 0x%X..0x%X (%ld trailing pad bytes)\n", AUDIO_BASE, base, fsz - (long)base);

    /* ---- optional SD_* names from SeData.bin ---------------------------- */
    mkdirp(outdir);
    char path[1024];
    Names *NM = calloc(total, sizeof *NM);
    if (sedata) {
        snprintf(path, sizeof path, "%s/se_map.csv", outdir);
        FILE *mapf = fopen(path, "w");
        load_names(sedata, hdrfile, counts, total, NM, mapf);
        if (mapf) fclose(mapf);
    }

    /* ---- one-big-file outputs ------------------------------------------ */
    FILE *bigwav = NULL, *bigcue = NULL, *bigdsp = NULL;
    uint64_t bigsamps = 0, bignib = 0, bigbytes = 0;
    if (big) {
        snprintf(path, sizeof path, "%s/all_waves.wav", outdir);
        bigwav = fopen(path, "wb");
        if (bigwav) { uint8_t z[44] = {0}; fwrite(z, 1, 44, bigwav); }
        snprintf(path, sizeof path, "%s/all_waves.dsp", outdir);
        bigdsp = fopen(path, "wb");
        if (bigdsp) { uint8_t z[0x60] = {0}; fwrite(z, 1, 0x60, bigdsp); }
        snprintf(path, sizeof path, "%s/all_waves.cue.csv", outdir);
        bigcue = fopen(path, "w");
        if (bigcue) fprintf(bigcue, "index,group,index_in_group,start_sample,length_samples,"
                                    "start_seconds,dsp_byte_offset,dsp_nibbles,se_name,coefficients\n");
    }
    snprintf(path, sizeof path, "%s/manifest.csv", outdir);
    FILE *mf = fopen(path, "w");
    if (mf) fprintf(mf, "index,group,index_in_group,file_offset,bytes,nibbles,samples,"
                        "seconds@%uHz,root_key,fine_tune,volume,pan,loop,coefficients,se_names\n", rate);

    int16_t *pcm = malloc(sizeof(int16_t) * 0x400000);
    uint64_t tot_samples = 0, tot_clip = 0;
    uint32_t written = 0, badframes = 0, totframes = 0;

    for (uint32_t i = 0; i < total; i++) {
        Entry *e = &E[i];
        if (only_group >= 0 && (long)e->group != only_group) continue;
        if (only_index >= 0 && (long)i != only_index) continue;
        if (e->offset + e->bytes > (uint32_t)fsz) {
            fprintf(stderr, "entry %u out of range, skipped\n", i); continue;
        }
        const uint8_t *src = d + e->offset;

        for (uint32_t p = 0; p + FRAME_BYTES <= e->bytes; p += FRAME_BYTES) {
            totframes++;
            if ((src[p] >> 4) > 7 || (src[p] & 0xF) > 12) badframes++;
        }

        uint32_t clip = 0;
        uint32_t n = dsp_decode(src, e->bytes, e->samples, e->coef, pcm, &clip);
        tot_samples += n; tot_clip += clip;

        char dir[1024];
        if (flat) snprintf(dir, sizeof dir, "%s", outdir);
        else { snprintf(dir, sizeof dir, "%s/group%02u", outdir, e->group); mkdirp(dir); }

        char stem[128];
        if (NM[i].cnt) {
            char safe[48]; int k = 0;
            for (const char *c = NM[i].n[0]; *c && k < 47; c++)
                safe[k++] = (isalnum((unsigned char)*c) || *c == '_') ? *c : '_';
            safe[k] = 0;
            snprintf(stem, sizeof stem, "se%04u_%s", i, safe);
        } else {
            snprintf(stem, sizeof stem, "se%04u", i);
        }

        if (want_wav) { snprintf(path, sizeof path, "%s/%s.wav", dir, stem); write_wav(path, pcm, n, rate); }
        if (want_dsp) { snprintf(path, sizeof path, "%s/%s.dsp", dir, stem); write_dsp(path, e, src, rate); }
        written++;

        if (bigwav) {
            if (bigcue) {
                fprintf(bigcue, "%u,%u,%u,%llu,%u,%.4f,%llu,%u,%s,", i, e->group,
                        e->index_in_group, (unsigned long long)bigsamps, n,
                        (double)bigsamps / rate,
                        (unsigned long long)(0x60 + bigbytes), e->nibbles,
                        NM[i].cnt ? NM[i].n[0] : "");
                for (int c = 0; c < 16; c++) fprintf(bigcue, "%d%s", e->coef[c], c == 15 ? "\n" : " ");
            }
            fwrite(pcm, 2, n, bigwav);
            bigsamps += n;
            if (bigdsp) { fwrite(src, 1, e->bytes, bigdsp); bignib += e->nibbles; bigbytes += e->bytes; }
        }

        if (mf) {
            fprintf(mf, "%u,%u,%u,0x%X,%u,%u,%u,%.4f,%u,%d,%u,%u,%u,",
                    i, e->group, e->index_in_group, e->offset, e->bytes, e->nibbles,
                    e->samples, (double)e->samples / rate, e->root_key, e->tune,
                    e->vol, e->pan, (e->flags & 0x200) ? 1 : 0);
            for (int c = 0; c < 16; c++) fprintf(mf, "%d ", e->coef[c]);
            fputc(',', mf);
            for (int q = 0; q < NM[i].cnt; q++) fprintf(mf, "%s%s", NM[i].n[q], q + 1 == NM[i].cnt ? "" : " ");
            fputc('\n', mf);
        }
    }
    if (mf) fclose(mf);
    if (bigcue) fclose(bigcue);
    if (bigwav) {
        uint8_t h[44]; uint32_t data = (uint32_t)(bigsamps * 2);
        memcpy(h, "RIFF", 4);        wl32(h + 4, 36 + data);
        memcpy(h + 8, "WAVEfmt ", 8); wl32(h + 16, 16);
        wl16(h + 20, 1); wl16(h + 22, 1);
        wl32(h + 24, rate); wl32(h + 28, rate * 2);
        wl16(h + 32, 2); wl16(h + 34, 16);
        memcpy(h + 36, "data", 4);   wl32(h + 40, data);
        fseek(bigwav, 0, SEEK_SET); fwrite(h, 1, 44, bigwav); fclose(bigwav);
        printf("one-big-file: all_waves.wav  %llu samples (%.1f s) + all_waves.cue.csv\n",
               (unsigned long long)bigsamps, (double)bigsamps / rate);
    }
    if (bigdsp) {
        /* Whole-bank ADPCM container.  A .dsp header carries exactly one set
         * of 16 coefficients, so this single header cannot describe all the
         * clips inside; it is filled in from the first clip and the real
         * per-clip coefficients are listed in all_waves.cue.csv.  Use it to
         * re-split the stream - all_waves.wav is the correct single-file
         * lossless rendering of the whole bank. */
        uint8_t h[0x60] = {0};
        wb32(h + 0x00, (uint32_t)(bigbytes / 8 * 14));
        wb32(h + 0x04, (uint32_t)(bigbytes * 2));
        wb32(h + 0x08, rate);
        wb16(h + 0x0C, 0);
        wb16(h + 0x0E, 0);
        wb32(h + 0x10, 2);
        wb32(h + 0x14, (uint32_t)(bigbytes * 2 - 1));
        for (int c = 0; c < 16; c++) wb16(h + 0x1C + 2 * c, (uint16_t)E[0].coef[c]);
        wb16(h + 0x3E, d[E[0].offset]);
        fseek(bigdsp, 0, SEEK_SET); fwrite(h, 1, sizeof h, bigdsp); fclose(bigdsp);
        printf("one-big-file: all_waves.dsp  %llu bytes of ADPCM (container, see cue csv)\n",
               (unsigned long long)bigbytes);
    }
    free(pcm); free(NM);

    printf("extracted %u samples to %s/\n", written, outdir);
    if (stats || badframes) {
        printf("frame validation: %u frames, %u invalid (%.4f%%)\n",
               totframes, badframes, totframes ? 100.0 * badframes / totframes : 0.0);
        printf("decode: %llu samples, %llu clipped (%.4f%%)\n",
               (unsigned long long)tot_samples, (unsigned long long)tot_clip,
               tot_samples ? 100.0 * (double)tot_clip / (double)tot_samples : 0.0);
    }
    free(E); free(d);
    return 0;
}
