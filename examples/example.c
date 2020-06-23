#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "panTompkins.h"

int main(int argc, char *argv[]) {
  FILE *fin;
  FILE *fout;

  #ifdef NDEBUG
  if (argc != 2 && argc != 3) {
    printf("Usage:\n  a.out <input file> [output file]\n");
    return EXIT_FAILURE;
  }

  fin = fopen(argv[1], "r");
  if (fin == NULL) {
    fprintf(stderr, "Failed to open input file '%s'(%d): %s.\n", argv[1],
            errno, strerror(errno));
    return EXIT_FAILURE;
  }

  if (argc == 2) {
    fout = stdout;
  } else {
    assert(argc == 3);
    fout = fopen(argv[2], "w+");
    if (fout == NULL) {
      fclose(fin);
      fprintf(stderr, "Failed to open output file '%s'(%d): %s.\n", argv[2],
              errno, strerror(errno));
      return EXIT_FAILURE;
    }
  }
  #else
  fin = fopen("./test_input.txt", "r");
  if (fin == NULL) {
    fprintf(stderr, "Failed to open input file '%s'(%d): %s.\n", argv[1], errno,
            strerror(errno));
    return EXIT_FAILURE;
  }

  fout = stdout;
  #endif

  init(fin, fout);
  panTompkins();

  fclose(fin);
  if (argc == 3) fclose(fout);

  return EXIT_SUCCESS;
}
