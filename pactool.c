/*
 * pactool - extractor & repacker for MMNT / "Rockman EXE Transmission" .pac archives
 * ===================================================================================
 *
 * FILE FORMAT (reverse-engineered & verified byte-exact against real samples)
 * -----------------------------------------------------------------------------
 * A .pac file is a flat, back-to-back sequence of "CAPR" chunks. There is no
 * outer table of contents, no padding and no alignment between chunks -- the
 * end of one chunk is the start of the next, and EOF ends the last one.
 *
 *   file   := chunk*
 *   chunk  := preamble(32 bytes) payload(size bytes)
 *
 *   preamble:
 *     +0x00  char[4]    magic        "CAPR"
 *     +0x04  uint32be   size         length of `payload`, NOT counting this
 *                                    32-byte preamble itself
 *     +0x08  uint8[8]   reserved     always zero in every sample seen
 *     +0x10  char[16]   name         resource name, NUL-padded (e.g.
 *                                    "EnvMap.pcp", "a01xxxxx.mpc", "card.pcp")
 *
 *   payload: `size` raw bytes. Structure depends on the extension embedded in
 *   `name` (.scn/.pcp texture+geometry blocks, .mpc skeleton+mesh blocks,
 *   map.dat/bg.dat/enemy.dat stage directories, ...). pactool treats this
 *   region as an opaque blob -- it round-trips it exactly without needing to
 *   understand it, which is what makes the repacker reliable. (For deep
 *   semantic parsing of what's *inside* a payload, see the companion Python
 *   script mmnt_pac_extract_full.py.)
 *
 * Verified against PackTest.pac (2 chunks), Fixed.pac (89 chunks) and
 * card.pac (1 chunk): every single chunk boundary in all three files lines
 * up exactly with this model, with zero leftover or overlap.
 *
 * WORKFLOW
 * --------
 *   pactool list    <input.pac>
 *   pactool extract <input.pac> <output_dir>
 *   pactool repack  <input_dir> <output.pac>
 *
 * extract writes each chunk's payload as its own file (e.g.
 * "000_EnvMap.pcp.bin"), plus a manifest.tsv that records the framing info
 * (name, reserved bytes) needed to rebuild the preamble. repack reads the
 * manifest, and for every entry re-derives `size` from the *current* length
 * of the payload file on disk -- so you can freely replace a payload file
 * with one of a different length (e.g. a re-encoded texture) and repack will
 * still produce a structurally valid .pac. With no edits at all,
 * extract + repack reproduces the original file byte-for-byte.
 *
 * Build:  gcc -std=c11 -O2 -Wall -Wextra -o pactool pactool.c
 */

#define _POSIX_C_SOURCE 200809L
#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#else
#include <unistd.h>
#define MKDIR(p) mkdir((p), 0755)
#endif

/* ------------------------------------------------------------------------ */
/* Format constants                                                         */
/* ------------------------------------------------------------------------ */

#define PAC_MAGIC      "CAPR"
#define PAC_PREAMBLE   32u   /* magic(4) + size(4) + reserved(8) + name(16) */
#define PAC_NAME_LEN   16u
#define PAC_RES_LEN    8u

typedef struct {
    char     name[PAC_NAME_LEN + 1];
    uint8_t  reserved[PAC_RES_LEN];
    uint32_t size;     /* payload length, NOT including the 32-byte preamble */
    uint64_t offset;   /* offset of the preamble (chunk start) in the file   */
} pac_chunk_t;

typedef struct {
    pac_chunk_t *chunks;
    size_t       count;
    size_t       cap;
    uint64_t     filesize;
    uint64_t     trailing_off; /* == filesize if nothing is left unparsed */
} pac_file_t;

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static void *xmalloc(size_t n) {
    void *p = malloc(n ? n : 1);
    if (!p) { fprintf(stderr, "pactool: error: out of memory (%zu bytes)\n", n); exit(1); }
    return p;
}

static void *xrealloc(void *old, size_t n) {
    void *p = realloc(old, n ? n : 1);
    if (!p) { fprintf(stderr, "pactool: error: out of memory (%zu bytes)\n", n); exit(1); }
    return p;
}

static uint32_t rd_u32be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

static void wr_u32be(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static void bytes_to_hex(const uint8_t *b, size_t n, char *out) {
    static const char hexd[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[i * 2]     = hexd[(b[i] >> 4) & 0xF];
        out[i * 2 + 1] = hexd[b[i] & 0xF];
    }
    out[n * 2] = '\0';
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int hex_to_bytes(const char *hex, uint8_t *out, size_t n) {
    if (strlen(hex) != n * 2) return -1;
    for (size_t i = 0; i < n; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return 0;
}

static int digits_needed(uint64_t n) {
    int d = 1;
    while (n >= 10) { n /= 10; d++; }
    return d;
}

static void sanitize_component(const char *in, char *out, size_t outsz) {
    size_t i = 0;
    for (; in[i] != '\0' && i + 1 < outsz; i++) {
        unsigned char c = (unsigned char)in[i];
        out[i] = (isalnum(c) || c == '.' || c == '_' || c == '-') ? (char)c : '_';
    }
    out[i] = '\0';
    if (i == 0) snprintf(out, outsz, "unnamed");
}

static int read_file(const char *path, uint8_t **out_buf, uint64_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); return -1; }
    if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return -1; }
    uint8_t *buf = NULL;
    if (sz > 0) {
        buf = (uint8_t *)xmalloc((size_t)sz);
        if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) { free(buf); fclose(f); return -1; }
    }
    fclose(f);
    *out_buf = buf;
    *out_len = (uint64_t)sz;
    return 0;
}

static int write_file(const char *path, const uint8_t *buf, uint64_t len) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    if (len > 0 && fwrite(buf, 1, (size_t)len, f) != (size_t)len) { fclose(f); return -1; }
    return fclose(f) == 0 ? 0 : -1;
}

/* mkdir -p */
static int ensure_dir(const char *path) {
    char buf[4096];
    size_t len = strlen(path);
    if (len == 0 || len >= sizeof(buf)) return -1;
    memcpy(buf, path, len + 1);
    for (size_t i = 1; i < len; i++) {
        if (buf[i] == '/') {
            buf[i] = '\0';
            if (MKDIR(buf) != 0 && errno != EEXIST) return -1;
            buf[i] = '/';
        }
    }
    if (MKDIR(buf) != 0 && errno != EEXIST) return -1;
    return 0;
}

static char *rtrim_newline(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = '\0';
    return s;
}

/* ------------------------------------------------------------------------ */
/* Core container parsing -- never hard-fails; degrades to "trailing raw    */
/* data" and a stderr warning if the input deviates from the model, so the */
/* tool stays safe/non-destructive on unexpected input.                     */
/* ------------------------------------------------------------------------ */

static void pac_push(pac_file_t *pf, pac_chunk_t c) {
    if (pf->count == pf->cap) {
        pf->cap = pf->cap ? pf->cap * 2 : 16;
        pf->chunks = (pac_chunk_t *)xrealloc(pf->chunks, pf->cap * sizeof(pac_chunk_t));
    }
    pf->chunks[pf->count++] = c;
}

static void parse_pac(const uint8_t *data, uint64_t filesize, pac_file_t *pf) {
    pf->chunks = NULL;
    pf->count = 0;
    pf->cap = 0;
    pf->filesize = filesize;

    uint64_t off = 0;
    while (off + PAC_PREAMBLE <= filesize) {
        if (memcmp(data + off, PAC_MAGIC, 4) != 0) {
            if (off == 0) {
                fprintf(stderr, "pactool: warning: no CAPR magic at offset 0 -- "
                                 "treating entire file as opaque trailing data\n");
            } else {
                fprintf(stderr, "pactool: warning: CAPR magic mismatch at offset 0x%llx "
                                 "after %zu chunk(s) -- preserving remaining %llu byte(s) "
                                 "as opaque trailing data\n",
                        (unsigned long long)off, pf->count,
                        (unsigned long long)(filesize - off));
            }
            break;
        }
        uint32_t size = rd_u32be(data + off + 4);
        uint64_t payload_off = off + PAC_PREAMBLE;
        if (payload_off + size > filesize) {
            fprintf(stderr, "pactool: warning: chunk %zu declares payload size %u but only "
                             "%llu byte(s) remain -- preserving remainder as opaque trailing data\n",
                    pf->count, size, (unsigned long long)(filesize - off));
            break;
        }

        pac_chunk_t c;
        memset(&c, 0, sizeof(c));
        memcpy(c.reserved, data + off + 8, PAC_RES_LEN);

        const uint8_t *namep = data + off + 0x10;
        size_t k = 0;
        while (k < PAC_NAME_LEN && namep[k] != 0) k++;
        memcpy(c.name, namep, k);
        c.name[k] = '\0';
        for (size_t j = k + 1; j < PAC_NAME_LEN; j++) {
            if (namep[j] != 0) {
                fprintf(stderr, "pactool: warning: chunk %zu ('%s') has non-zero bytes after "
                                 "its name terminator -- those bytes will be lost on repack\n",
                        pf->count, c.name);
                break;
            }
        }

        c.size = size;
        c.offset = off;
        pac_push(pf, c);

        off = payload_off + size;
    }
    pf->trailing_off = off;
}

static void pac_file_free(pac_file_t *pf) {
    free(pf->chunks);
    pf->chunks = NULL;
}

/* ------------------------------------------------------------------------ */
/* Commands                                                                  */
/* ------------------------------------------------------------------------ */

static int cmd_list(int argc, char **argv) {
    if (argc != 1) { fprintf(stderr, "usage: pactool list <file.pac>\n"); return 1; }
    const char *infile = argv[0];

    uint8_t *data; uint64_t len;
    if (read_file(infile, &data, &len) != 0) {
        fprintf(stderr, "pactool: error: cannot read '%s': %s\n", infile, strerror(errno));
        return 1;
    }

    pac_file_t pf;
    parse_pac(data, len, &pf);

    printf("%-5s  %-16s  %12s  %12s  %12s\n", "idx", "name", "size", "offset", "end");
    for (size_t i = 0; i < pf.count; i++) {
        pac_chunk_t *c = &pf.chunks[i];
        printf("%-5zu  %-16s  %12u  0x%010llx  0x%010llx\n",
               i, c->name, c->size,
               (unsigned long long)c->offset,
               (unsigned long long)(c->offset + PAC_PREAMBLE + c->size));
    }
    if (pf.trailing_off < len) {
        printf("trailing (unparsed): %llu byte(s) at offset 0x%010llx\n",
               (unsigned long long)(len - pf.trailing_off), (unsigned long long)pf.trailing_off);
    }
    printf("\n%zu chunk(s), %llu byte(s) total\n", pf.count, (unsigned long long)len);

    free(data);
    pac_file_free(&pf);
    return 0;
}

static int cmd_extract(int argc, char **argv) {
    if (argc != 2) { fprintf(stderr, "usage: pactool extract <file.pac> <output_dir>\n"); return 1; }
    const char *infile = argv[0];
    const char *outdir = argv[1];

    uint8_t *data; uint64_t len;
    if (read_file(infile, &data, &len) != 0) {
        fprintf(stderr, "pactool: error: cannot read '%s': %s\n", infile, strerror(errno));
        return 1;
    }

    pac_file_t pf;
    parse_pac(data, len, &pf);

    if (ensure_dir(outdir) != 0) {
        fprintf(stderr, "pactool: error: cannot create output directory '%s': %s\n", outdir, strerror(errno));
        free(data); pac_file_free(&pf);
        return 1;
    }

    char manifest_path[4096];
    snprintf(manifest_path, sizeof(manifest_path), "%s/manifest.tsv", outdir);
    FILE *mf = fopen(manifest_path, "wb");
    if (!mf) {
        fprintf(stderr, "pactool: error: cannot write manifest '%s': %s\n", manifest_path, strerror(errno));
        free(data); pac_file_free(&pf);
        return 1;
    }

    const char *base = strrchr(infile, '/');
    base = base ? base + 1 : infile;

    fprintf(mf, "# pactool manifest v1\n");
    fprintf(mf, "# source: %s\n", base);
    fprintf(mf, "# source_size: %llu\n", (unsigned long long)len);
    fprintf(mf, "# chunk_count: %zu\n", pf.count);
    fprintf(mf, "# columns: index<TAB>name<TAB>reserved_hex<TAB>data_file\n#\n");

    int width = digits_needed(pf.count > 0 ? pf.count - 1 : 0);
    if (width < 3) width = 3;

    for (size_t i = 0; i < pf.count; i++) {
        pac_chunk_t *c = &pf.chunks[i];

        char safe[64];
        sanitize_component(c->name, safe, sizeof(safe));
        char dfile[160];
        snprintf(dfile, sizeof(dfile), "%0*zu_%s.bin", width, i, safe);
        char dpath[4096];
        snprintf(dpath, sizeof(dpath), "%s/%s", outdir, dfile);

        if (write_file(dpath, data + c->offset + PAC_PREAMBLE, c->size) != 0) {
            fprintf(stderr, "pactool: error: cannot write '%s': %s\n", dpath, strerror(errno));
            fclose(mf); free(data); pac_file_free(&pf);
            return 1;
        }

        char hexres[PAC_RES_LEN * 2 + 1];
        bytes_to_hex(c->reserved, PAC_RES_LEN, hexres);
        fprintf(mf, "%0*zu\t%s\t%s\t%s\n", width, i, c->name, hexres, dfile);
        printf("  [%0*zu] %-16s  %10u bytes  -> %s\n", width, i, c->name, c->size, dfile);
    }

    if (pf.trailing_off < len) {
        uint64_t tlen = len - pf.trailing_off;
        char tpath[4096];
        snprintf(tpath, sizeof(tpath), "%s/_trailing.bin", outdir);
        if (write_file(tpath, data + pf.trailing_off, tlen) != 0) {
            fprintf(stderr, "pactool: error: cannot write '%s': %s\n", tpath, strerror(errno));
            fclose(mf); free(data); pac_file_free(&pf);
            return 1;
        }
        fprintf(mf, "@TRAILING\t_trailing.bin\n");
        printf("  [trailing]                       %10llu bytes  -> _trailing.bin\n", (unsigned long long)tlen);
    }

    fclose(mf);
    printf("\nExtracted %zu chunk(s) from '%s' -> %s/\n", pf.count, infile, outdir);
    printf("Manifest: %s\n", manifest_path);

    free(data);
    pac_file_free(&pf);
    return 0;
}

static int cmd_repack(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: pactool repack <input_dir> <output.pac> [manifest_file]\n");
        return 1;
    }
    const char *indir = argv[0];
    const char *outfile = argv[1];

    char manifest_path[4096];
    if (argc == 3) snprintf(manifest_path, sizeof(manifest_path), "%s", argv[2]);
    else           snprintf(manifest_path, sizeof(manifest_path), "%s/manifest.tsv", indir);

    FILE *mf = fopen(manifest_path, "rb");
    if (!mf) {
        fprintf(stderr, "pactool: error: cannot open manifest '%s': %s\n", manifest_path, strerror(errno));
        return 1;
    }
    FILE *of = fopen(outfile, "wb");
    if (!of) {
        fprintf(stderr, "pactool: error: cannot create output '%s': %s\n", outfile, strerror(errno));
        fclose(mf);
        return 1;
    }

    char line[4096];
    size_t lineno = 0;
    size_t nchunks = 0;
    uint64_t totalbytes = 0;
    char trailing_file[1024];
    trailing_file[0] = '\0';
    int ok = 1;

    while (fgets(line, sizeof(line), mf)) {
        lineno++;
        rtrim_newline(line);
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '#') continue;

        if (strncmp(p, "@TRAILING", 9) == 0) {
            char *tab = strchr(p, '\t');
            if (!tab) {
                fprintf(stderr, "pactool: error: manifest line %zu: malformed @TRAILING entry\n", lineno);
                ok = 0; break;
            }
            snprintf(trailing_file, sizeof(trailing_file), "%s", tab + 1);
            continue;
        }

        char *fields[4]; int nf = 0;
        char *save = NULL;
        char *tok = strtok_r(p, "\t", &save);
        while (tok && nf < 4) { fields[nf++] = tok; tok = strtok_r(NULL, "\t", &save); }
        if (nf != 4) {
            fprintf(stderr, "pactool: error: manifest line %zu: expected 4 tab-separated fields, found %d\n", lineno, nf);
            ok = 0; break;
        }

        const char *idx_str = fields[0];
        const char *name    = fields[1];
        const char *reshex  = fields[2];
        const char *dfile   = fields[3];

        if (strlen(name) > PAC_NAME_LEN) {
            fprintf(stderr, "pactool: error: manifest line %zu: name '%s' exceeds %u bytes\n", lineno, name, PAC_NAME_LEN);
            ok = 0; break;
        }
        uint8_t reserved[PAC_RES_LEN];
        if (hex_to_bytes(reshex, reserved, PAC_RES_LEN) != 0) {
            fprintf(stderr, "pactool: error: manifest line %zu: malformed reserved-bytes hex '%s'\n", lineno, reshex);
            ok = 0; break;
        }

        char dpath[4096];
        snprintf(dpath, sizeof(dpath), "%s/%s", indir, dfile);
        uint8_t *buf; uint64_t blen;
        if (read_file(dpath, &buf, &blen) != 0) {
            fprintf(stderr, "pactool: error: manifest line %zu: cannot read data file '%s': %s\n",
                    lineno, dpath, strerror(errno));
            ok = 0; break;
        }
        if (blen > 0xFFFFFFFFull) {
            fprintf(stderr, "pactool: error: manifest line %zu: data file '%s' is too large (%llu bytes, max %u)\n",
                    lineno, dpath, (unsigned long long)blen, UINT32_MAX);
            free(buf); ok = 0; break;
        }

        uint8_t preamble[PAC_PREAMBLE];
        memset(preamble, 0, sizeof(preamble));
        memcpy(preamble, "CAPR", 4);
        wr_u32be(preamble + 4, (uint32_t)blen);
        memcpy(preamble + 8, reserved, PAC_RES_LEN);
        memcpy(preamble + 0x10, name, strlen(name));

        if (fwrite(preamble, 1, PAC_PREAMBLE, of) != PAC_PREAMBLE ||
            (blen > 0 && fwrite(buf, 1, (size_t)blen, of) != (size_t)blen)) {
            fprintf(stderr, "pactool: error: write failure on '%s'\n", outfile);
            free(buf); ok = 0; break;
        }

        printf("  [%s] %-16s  %10llu bytes  <- %s\n", idx_str, name, (unsigned long long)blen, dfile);

        free(buf);
        nchunks++;
        totalbytes += PAC_PREAMBLE + blen;
    }

    if (ok && trailing_file[0]) {
        char dpath[4096];
        snprintf(dpath, sizeof(dpath), "%s/%s", indir, trailing_file);
        uint8_t *buf; uint64_t blen;
        if (read_file(dpath, &buf, &blen) != 0) {
            fprintf(stderr, "pactool: error: cannot read trailing data file '%s': %s\n", dpath, strerror(errno));
            ok = 0;
        } else {
            if (blen > 0 && fwrite(buf, 1, (size_t)blen, of) != (size_t)blen) {
                fprintf(stderr, "pactool: error: write failure on '%s'\n", outfile);
                ok = 0;
            } else {
                printf("  [trailing]                       %10llu bytes  <- %s\n", (unsigned long long)blen, trailing_file);
                totalbytes += blen;
            }
            free(buf);
        }
    }

    fclose(mf);
    fclose(of);

    if (!ok) {
        remove(outfile);
        return 1;
    }

    printf("\nRepacked %zu chunk(s), %llu byte(s) -> %s\n", nchunks, (unsigned long long)totalbytes, outfile);

    /* Self-verification: re-parse what we just wrote. */
    {
        uint8_t *vdata; uint64_t vlen;
        if (read_file(outfile, &vdata, &vlen) == 0) {
            pac_file_t vpf;
            parse_pac(vdata, vlen, &vpf);
            if (vpf.count == nchunks) {
                uint64_t trail = vlen - vpf.trailing_off;
                printf("Verification OK: output re-parses as %zu chunk(s)", vpf.count);
                if (trail > 0) printf(" + %llu trailing byte(s)", (unsigned long long)trail);
                printf(".\n");
            } else {
                printf("Verification WARNING: re-parsed %zu chunk(s), expected %zu -- "
                       "output may not be well-formed.\n", vpf.count, nchunks);
            }
            free(vdata);
            pac_file_free(&vpf);
        }
    }

    return 0;
}

/* ------------------------------------------------------------------------ */

static void usage(const char *prog) {
    fprintf(stderr,
        "pactool - MMNT/\"Rockman EXE Transmission\" .pac archive extractor & repacker\n"
        "\n"
        "usage:\n"
        "  %s list    <input.pac>\n"
        "  %s extract <input.pac> <output_dir>\n"
        "  %s repack  <input_dir> <output.pac> [manifest_file]\n",
        prog, prog, prog);
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(argv[0]); return 1; }
    const char *cmd = argv[1];

    if (strcmp(cmd, "extract") == 0) return cmd_extract(argc - 2, argv + 2);
    if (strcmp(cmd, "repack")  == 0) return cmd_repack(argc - 2, argv + 2);
    if (strcmp(cmd, "list")    == 0) return cmd_list(argc - 2, argv + 2);
    if (strcmp(cmd, "-h") == 0 || strcmp(cmd, "--help") == 0 || strcmp(cmd, "help") == 0) {
        usage(argv[0]);
        return 0;
    }

    fprintf(stderr, "pactool: unknown command '%s'\n\n", cmd);
    usage(argv[0]);
    return 1;
}
