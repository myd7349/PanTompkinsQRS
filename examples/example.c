#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "import_data.h"
#include "panTompkins.h"

int read_int(const char *line, size_t line_no, void *data) {
  char *end;
  long result;

  assert(line != NULL);
  assert(data != NULL);

  result = strtol(line, &end, 10);
  if (errno == ERANGE || line == end) return 0;

  if (result < INT_MIN || result > INT_MAX) return 0;

  *(int *)data = (int)result;
  return 1;
}

int main(int argc, char *argv[]) {
  FILE *fin;
  size_t col;
  size_t i;
  int *data;
  FILE *fout;
  pan_tompkins_context_t qrs_detector;

#ifdef NDEBUG
  if (argc != 2 && argc != 3) {
    printf("Usage:\n  a.out <input file> [output file]\n");
    return EXIT_FAILURE;
  }

  fin = fopen(argv[1], "r");
  if (fin == NULL) {
    fprintf(stderr, "Failed to open input file '%s'(%d): %s.\n", argv[1], errno,
            strerror(errno));
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

  data = import_data(fin, sizeof(int), 0, read_int, &col);
  if (data != NULL) {
    qrs_detector = pan_tompkins_init(360, 22, 20, 600);
    if (qrs_detector != NULL) {
      for (i = 0; i < col; ++i)
        fprintf(fout, "%d\n", pan_tompkins_run(qrs_detector, data[i]));

      pan_tompkins_free(&qrs_detector);
    }

    free(&data);
  }

  fclose(fin);
  if (argc == 3) fclose(fout);

  return EXIT_SUCCESS;
}
