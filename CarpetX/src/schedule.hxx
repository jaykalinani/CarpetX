#ifndef SCHEDULE_HXX
#define SCHEDULE_HXX

#include "driver.hxx"
#include "loop.hxx"

#include <cctk.h>
#include <cctk_Schedule.h>
#undef copysign
#undef fpclassify
#undef isfinite
#undef isinf
#undef isnan
#undef isnormal
#undef signbit

#include <algorithm>
#include <array>

namespace CarpetX {
using namespace Loop;
using namespace std;
using std::max;
using std::min;

int Initialise(tFleshConfig *config);
int Evolve(tFleshConfig *config);
int Shutdown(tFleshConfig *config);

int SyncGroupsByDirI(const cGH *restrict cctkGH, int numgroups,
                     const int *groups, const int *directions);

int CallFunction(void *function, cFunctionData *attribute, void *data);

int GroupStorageIncrease(const cGH *cctkGH, int n_groups, const int *groups,
                         const int *tls, int *status);
int GroupStorageDecrease(const cGH *cctkGH, int n_groups, const int *groups,
                         const int *tls, int *status);
int EnableGroupStorage(const cGH *cctkGH, const char *groupname);
int DisableGroupStorage(const cGH *cctkGH, const char *groupname);

////////////////////////////////////////////////////////////////////////////////

// This global variable passes the current cctkGH to CactusAmrCore.
// (When it is null, then CactusAmrCore does not call any scheduled
// functions. This is used early during startup.)
extern cGH *saved_cctkGH;

// Whether CallFunction traverses all levels (-1) or just one specific level
// (>=0)
extern int current_level;

////////////////////////////////////////////////////////////////////////////////

// Like an MFIter, but does not support iteration, instead it can be copied
struct MFPointer {
  int m_index;
  Box m_fabbox;
  Box m_growntilebox;
  Box m_validbox;
  IntVect m_nGrowVect;

  MFPointer() = delete;
  MFPointer(const MFPointer &) = default;
  MFPointer(MFPointer &&) = default;
  MFPointer &operator=(const MFPointer &) = default;
  MFPointer &operator=(MFPointer &&) = default;
  MFPointer(const MFIter &mfi)
      : m_index(mfi.index()), m_fabbox(mfi.fabbox()),
        m_growntilebox(mfi.growntilebox()), m_validbox(mfi.validbox()),
        m_nGrowVect(mfi.theFabArrayBase().nGrowVect()) {}

  constexpr int index() const noexcept { return m_index; }
  constexpr Box fabbox() const noexcept { return m_fabbox; }
  constexpr Box growntilebox() const noexcept { return m_growntilebox; }
  constexpr Box validbox() const noexcept { return m_validbox; }
  constexpr IntVect nGrowVect() const noexcept { return m_nGrowVect; }
};

struct GridDesc : GridDescBase {

  GridDesc() = delete;
  GridDesc(const GHExt::LevelData &leveldata, const MFPointer &mfp);
  GridDesc(const cGH *cctkGH) : GridDescBase(cctkGH) {}
};

// TODO: remove this
struct GridPtrDesc : GridDesc {
  Dim3 cactus_offset;

  GridPtrDesc() = delete;
  GridPtrDesc(const GHExt::LevelData &leveldata, const MFPointer &mfp);

  template <typename T> T *ptr(const Array4<T> &vars, int vi) const {
    return vars.ptr(cactus_offset.x, cactus_offset.y, cactus_offset.z, vi);
  }
  template <typename T>
  T &idx(const Array4<T> &vars, int i, int j, int k, int vi) const {
    return vars(cactus_offset.x + i, cactus_offset.y + i, cactus_offset.z + j,
                vi);
  }
};

struct GridPtrDesc1 : GridDesc {
  Dim3 cactus_offset;
  array<int, dim> gimin, gimax;
  array<int, dim> gash;

  GridPtrDesc1() = delete;
  GridPtrDesc1(const GridPtrDesc1 &) = delete;
  GridPtrDesc1 &operator=(const GridPtrDesc1 &) = delete;

  GridPtrDesc1(const GHExt::LevelData::GroupData &groupdata,
               const MFPointer &mfp);

  template <typename T> T *ptr(const Array4<T> &vars, int vi) const {
    return vars.ptr(cactus_offset.x + gimin[0], cactus_offset.y + gimin[1],
                    cactus_offset.z + gimin[2], vi);
  }
  template <typename T>
  T &idx(const Array4<T> &vars, int i, int j, int k, int vi) const {
    return vars(cactus_offset.x + gimin[0] + i, cactus_offset.y + gimin[1] + i,
                cactus_offset.z + gimin[2] + j, vi);
  }

  template <typename T> GF3D1<T> gf3d(const Array4<T> &vars, int vi) const {
    return GF3D1<T>(ptr(vars, vi), gimin, gimax, gash);
  }
};

bool in_local_mode(const cGH *restrict cctkGH);
bool in_level_mode(const cGH *restrict cctkGH);
bool in_global_mode(const cGH *restrict cctkGH);
bool in_meta_mode(const cGH *restrict cctkGH);

void error_if_invalid(const GHExt::LevelData ::GroupData &grouppdata, int vi,
                      int tl, const valid_t &required,
                      const function<string()> &msg);
void warn_if_invalid(const GHExt::LevelData ::GroupData &grouppdata, int vi,
                     int tl, const valid_t &required,
                     const function<string()> &msg);
void poison_invalid(const GHExt::LevelData::GroupData &groupdata, int vi,
                    int tl);
void check_valid(const GHExt::LevelData::GroupData &groupdata, int vi, int tl,
                 const function<string()> &msg);

void error_if_invalid(const GHExt::GlobalData::ScalarGroupData &groupdata,
                      int vi, int tl, const valid_t &required,
                      const function<string()> &msg);
void warn_if_invalid(const GHExt::GlobalData::ScalarGroupData &groupdata,
                     int vi, int tl, const valid_t &required,
                     const function<string()> &msg);
void poison_invalid(const GHExt::GlobalData::ScalarGroupData &groupdata, int vi,
                    int tl);
void check_valid(const GHExt::GlobalData::ScalarGroupData &groupdata, int vi,
                 int tl, const function<string()> &msg);

} // namespace CarpetX

#endif // #ifndef SCHEDULE_HXX
