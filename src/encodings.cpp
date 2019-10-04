#include "jpsl/encodings.hpp"

using namespace std;
using namespace JPSL;

// 1D coordinate changes
Dir JPSL::fnd_id(const Dir & dir) { return dir; }
Dir JPSL::fnd_neg(const Dir & dir) { return -dir; }
Dir JPSL::fnd_fp_x(const Dir & dir) { return Dir(-dir.dx(), dir.dy(), dir.dz()); }
Dir JPSL::fnd_fp_y(const Dir & dir) { return Dir(dir.dx(), -dir.dy(), dir.dz()); }
Dir JPSL::fnd_fp_z(const Dir & dir) { return Dir(dir.dx(), dir.dy(), -dir.dz()); }
Dir JPSL::fnd_fp_xz(const Dir & dir) { return Dir(-dir.dx(), dir.dy(), -dir.dz()); }
Dir JPSL::fnd_fp_xy(const Dir & dir) { return Dir(-dir.dx(), -dir.dy(), dir.dz()); }
Dir JPSL::fnd_fp_yz(const Dir & dir) { return Dir(dir.dx(), -dir.dy(), -dir.dz()); }
Dir JPSL::fnd_ch_xy(const Dir & dir) { return Dir(dir.dy(), dir.dx(), dir.dz()); }
Dir JPSL::fnd_ch_xz(const Dir & dir) { return Dir(dir.dz(), dir.dy(), dir.dx()); }
Dir JPSL::fnd_ch_yz(const Dir & dir) { return Dir(dir.dx(), dir.dz(), dir.dy()); }
Dir JPSL::fnd_chfp_xy(const Dir & dir) { return Dir(-dir.dy(), -dir.dx(), dir.dz()); }
Dir JPSL::fnd_chfp_xz(const Dir & dir) { return Dir(-dir.dz(), dir.dy(), -dir.dx()); }
Dir JPSL::fnd_chfp_yz(const Dir & dir) { return Dir(dir.dx(), -dir.dz(), -dir.dy()); }
Dir JPSL::fnd_fp_x_ch_yz(const Dir & dir) { return Dir(-dir.dx(), dir.dz(), dir.dy()); }
Dir JPSL::fnd_fp_y_ch_xz(const Dir & dir) { return Dir(dir.dz(), -dir.dy(), dir.dx()); }
Dir JPSL::fnd_fp_x_chfp_yz(const Dir & dir) { return Dir(-dir.dx(), -dir.dz(), -dir.dy()); }
Dir JPSL::fnd_fp_y_chfp_xz(const Dir & dir) { return Dir(-dir.dz(), -dir.dy(), -dir.dx()); }

Dir(*JPSL::standardize_dir(const Dir & d0))(const Dir &) {
  // coordinate change so that d0 becomes one of [-1 0 0] [-1 -1 0] or [-1 -1 -1]
  // and s.t. coordinate change is its own inverse

  switch(d0.order()) {
    case 1:
      if (d0.dx() == -1)  // identity 
        return &fnd_id;
      if (d0.dx() == 1)   // mirror
        return &fnd_neg;
      if (d0.dy() == -1)  // change x,y
        return &fnd_ch_xy;
      if (d0.dy() == 1)   // change-flip x,y
        return &fnd_chfp_xy;
      if (d0.dz() == -1)  // change x,z
        return &fnd_ch_xz;
      if (d0.dz() == 1)   // change-flip x,z
        return &fnd_chfp_xz;
      break;
    case 2:
      if (d0.dx() == -1 && d0.dy() == -1) // identity 
        return &fnd_id;
      if (d0.dx() == 1 && d0.dy() == 1)   // flip all
        return &fnd_neg;
      if (d0.dx() == 1 && d0.dy() == -1)  // flip x
        return &fnd_fp_x;
      if (d0.dx() == -1 && d0.dy() == 1)  // flip y
        return &fnd_fp_y;

      if (d0.dx() == -1 && d0.dz() == -1) // change y/z 
        return &fnd_ch_yz;
      if (d0.dx() == -1 && d0.dz() == 1)  // change-flip y/z
        return &fnd_chfp_yz;
      if (d0.dx() == 1 && d0.dz() == 1)   // flip x, change-flip y/z
        return &fnd_fp_x_chfp_yz;
      if (d0.dx() == 1 && d0.dz() == -1)  // flip x, change y/z
        return &fnd_fp_x_ch_yz;

      if (d0.dy() == -1 && d0.dz() == -1)  // change x/z 
        return &fnd_ch_xz;
      if (d0.dy() == -1 && d0.dz() == 1)   // change-flip x/z 
        return &fnd_chfp_xz;
      if (d0.dy() == 1 && d0.dz() == 1)    // flip y, change-flip x/z
        return &fnd_fp_y_chfp_xz;
      if (d0.dy() == 1 && d0.dz() == -1)   // flip y, change x/z
        return &fnd_fp_y_ch_xz;
      break;
    case 3:
      if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == -1)  // identity 
        return &fnd_id;
      if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == 1)     // flip all 
        return &fnd_neg;

      if (d0.dx() == 1 && d0.dy() == 1 && d0.dz() == -1)  // flip x,y 
        return &fnd_fp_xy;
      if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == 1)  // flip x,z 
        return &fnd_fp_xz;
      if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == 1)  // flip y,z
        return &fnd_fp_yz;

      if (d0.dx() == 1 && d0.dy() == -1 && d0.dz() == -1)  // flip x
        return &fnd_fp_x;
      if (d0.dx() == -1 && d0.dy() == 1 && d0.dz() == -1)  // flip y
        return &fnd_fp_y;
      if (d0.dx() == -1 && d0.dy() == -1 && d0.dz() == 1)  // flip z
        return &fnd_fp_z;
      break;
  }

  return &fnd_id;
}
