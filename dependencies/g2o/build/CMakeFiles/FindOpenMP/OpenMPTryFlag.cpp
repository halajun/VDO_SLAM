
#include <omp.h>
int main() {
#ifdef _OPENMP
  omp_get_max_threads();
  return 0;
#else
  breaks_on_purpose
#endif
}
