/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef VOXEL_GRID_VOXEL_GRID_H
#define VOXEL_GRID_VOXEL_GRID_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>

/**
 * @class VoxelGrid
 * @brief A 3D grid structure that stores points as an integer array.
 *        X and Y index the array and Z selects which bit of the integer
 *        is used giving a limit of Z_VOXELS vertical cells.
 */
namespace voxel_grid
{

// TODO(ahoy): make these better than just defines?
#define Z_VOXELS 32
typedef uint64_t VoxelDataColumn;
typedef uint32_t VoxelMarkedColumn;

enum VoxelStatus {
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

class VoxelGrid
{
public:
  /**
   * @brief  Constructor for a voxel grid
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= Z_VOXELS are supported
   */
  VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  ~VoxelGrid();

  /**
   * @brief  Resizes a voxel grid to the desired size
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= Z_VOXELS are supported
   */
  void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  inline unsigned int getZVoxelCount()
  {
    return Z_VOXELS;
  }

  void reset();
  VoxelDataColumn* getData() { return data_; }

  inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    VoxelDataColumn full_mask = ((VoxelDataColumn)1LL<<z<<Z_VOXELS) | (1LL<<z);
    data_[y * size_x_ + x] |= full_mask; //clear unknown and mark cell
  }

  inline bool markVoxelInMap(unsigned int x, unsigned int y, unsigned int z, unsigned int marked_threshold)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return false;
    }

    int index = y * size_x_ + x;
    VoxelDataColumn* col = &data_[index];
    VoxelDataColumn full_mask = ((VoxelDataColumn)1LL<<z<<Z_VOXELS) | (1LL<<z);
    *col |= full_mask; //clear unknown and mark cell

    VoxelMarkedColumn marked_bits = *col>>Z_VOXELS;

    //make sure the number of bits in each is below our thesholds
    return !bitsBelowThreshold(marked_bits, marked_threshold);
  }

  inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    VoxelDataColumn full_mask = ((VoxelDataColumn)1LL<<z<<Z_VOXELS) | (1LL<<z);
    data_[y * size_x_ + x] &= ~(full_mask); //clear unknown and clear cell
  }

  inline void clearVoxelColumn(unsigned int index)
  {
    ROS_ASSERT(index < size_x_ * size_y_);
    data_[index] = 0;
  }

  inline void clearVoxelInMap(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    int index = y * size_x_ + x;
    VoxelDataColumn* col = &data_[index];
    VoxelDataColumn full_mask = ((VoxelDataColumn)1LL<<z<<Z_VOXELS) | (1LL<<z);
    *col &= ~(full_mask); //clear unknown and clear cell

    VoxelDataColumn unknown_bits = VoxelMarkedColumn(*col>>Z_VOXELS) ^ VoxelMarkedColumn(*col);
    VoxelDataColumn marked_bits = *col>>Z_VOXELS;

    //make sure the number of bits in each is below our thesholds
    if (bitsBelowThreshold(unknown_bits, 1) && bitsBelowThreshold(marked_bits, 1))
    {
      costmap[index] = 0;
    }
  }

  inline bool bitsBelowThreshold(VoxelDataColumn n, unsigned int bit_threshold)
  {
    unsigned int bit_count;
    for (bit_count = 0; n;)
    {
      ++bit_count;
      if (bit_count > bit_threshold)
      {
        return false;
      }
      n &= n - 1LL; //clear the least significant bit set
    }
    return true;
  }

  static inline unsigned int numBits(VoxelDataColumn n)
  {
    unsigned int bit_count;
    for (bit_count = 0; n; ++bit_count)
    {
      n &= n - 1LL; //clear the least significant bit set
    }
    return bit_count;
  }

  static VoxelStatus getVoxel(
    unsigned int x, unsigned int y, unsigned int z,
    unsigned int size_x, unsigned int size_y, unsigned int size_z, const VoxelDataColumn* data)
  {
    if (x >= size_x || y >= size_y || z >= size_z)
    {
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    VoxelDataColumn full_mask = ((VoxelDataColumn)1LL<<z<<Z_VOXELS) | (1LL<<z);
    VoxelDataColumn result = data[y * size_x + x] & full_mask;
    // TODO(ahoy): no need to count bits here, can just check them
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if (bits < 2)
    {
      if (bits < 1)
      {
        return FREE;
      }
      return UNKNOWN;
    }
    return MARKED;
  }

  void markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d,
                           unsigned int unknown_threshold, unsigned int mark_threshold,
                           unsigned char free_cost = 0, unsigned char unknown_cost = 255, unsigned int max_length = UINT_MAX);

  VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);

  //Are there any obstacles at that (x, y) location in the grid?
  VoxelStatus getVoxelColumn(unsigned int x, unsigned int y,
                             unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0);

  void printVoxelGrid();
  void printColumnGrid();
  unsigned int sizeX();
  unsigned int sizeY();
  unsigned int sizeZ();

  #define FP_BITS 4

  template <class ActionType>
  inline void raytraceLine(
    ActionType at, double x0, double y0, double z0,
    double x1, double y1, double z1, unsigned int max_length = UINT_MAX)
  {
    int dx = int((x1 - x0) * (1<<FP_BITS));
    int dy = int((y1 - y0) * (1<<FP_BITS));
    int dz = int((z1 - z0) * (1<<FP_BITS));

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;
    int offset_dz = sign(dz);

    VoxelDataColumn z_mask = ((1LL << Z_VOXELS) | 1LL) << (unsigned int)z0;
    unsigned int offset = (unsigned int)y0 * size_x_ + (unsigned int)x0;

    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0,  max_length / dist);

    double ex = x0 - (int)x0;
    if (dx < 0)
    {
      ex = 1.0 - ex;
    }

    double ey = y0 - (int)y0;
    if (dy < 0)
    {
      ey = 1.0 - ey;
    }

    double ez = z0 - (int)z0;
    if (dz < 0)
    {
      ez = 1.0 - ez;
    }

    //is x dominant
    if (abs_dx >= max(abs_dy, abs_dz))
    {
      // Find out how far we have to go along major axis to get to the middle
      // (which is where the algorithm assumes we are)
      double d_ex_m = 0.5 - ex;

      // Now advance the other axes' errors by that times the slope
      double y_at_m = ey + ((d_ex_m * abs_dy) / abs_dx);
      double z_at_m = ez + ((d_ex_m * abs_dz) / abs_dx);

      // Set errors to scale assuming that starting point in the cell
      int error_y = abs_dx * y_at_m;
      int error_z = abs_dx * z_at_m;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask, (unsigned int)(scale * std::fabs(x1 - x0) + 0.5));
      return;
    }

    //y is dominant
    if (abs_dy >= abs_dz)
    {
      // Find out how far we have to go along major axis to get to the middle
      // (which is where the algorithm assumes we are)
      double d_ey_m = 0.5 - ey;

      // Now advance the other axes' errors by that times the slope
      double x_at_m = ex + ((d_ey_m * abs_dx) / abs_dy);
      double z_at_m = ez + ((d_ey_m * abs_dz) / abs_dy);

      // Set errors to scale assuming that starting point in the cell
      int error_x = abs_dy * x_at_m;
      int error_z = abs_dy * z_at_m;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask, (unsigned int)(scale * std::fabs(y1 - y0) + 0.5));
      return;
    }

    //otherwise, z is dominant
    // Find out how far we have to go along major axis to get to the middle
    // (which is where the algorithm assumes we are)
    double d_ez_m = 0.5 - ez;

    // Now advance the other axes' errors by that times the slope
    double x_at_m = ex + ((d_ez_m * abs_dx) / abs_dz);
    double y_at_m = ey + ((d_ez_m * abs_dy) / abs_dz);

    // Set errors to scale assuming that starting point in the cell
    int error_x = abs_dz * x_at_m;
    int error_y = abs_dz * y_at_m;

    bresenham3D(at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask, (unsigned int)(scale * std::fabs(z1 - z0) + 0.5));
  }

private:
  //the real work is done here... 3D bresenham implementation
  template <class ActionType, class OffA, class OffB, class OffC>
  inline void bresenham3D(
    ActionType at, OffA off_a, OffB off_b, OffC off_c,
    unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
    int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
    VoxelDataColumn &z_mask, unsigned int max_length = UINT_MAX)
  {
    int errorprev_b = error_b;
    int errorprev_c = error_c;

    for (unsigned int i = 0; i < max_length; ++i)
    {
      at(offset, z_mask);
      off_a(offset_a);
      error_b += abs_db;
      error_c += abs_dc;

      if (((unsigned int)error_b >= abs_da) && ((unsigned int)error_c >= abs_da))
      {
        if (error_b > error_c)
        {
          // If b cut over first, do it's stuff first
          off_b(offset_b);
          error_b -= abs_da;

          if (error_b + errorprev_b < abs_da)
          {
            off_b(-offset_b);
            at(offset, z_mask);
            off_b(offset_b);
          }
          else
          {
            off_a(-offset_a);
            at(offset, z_mask);
            off_a(offset_a);
          }

          off_c(offset_c);
          error_c -= abs_da;

          if (error_c + errorprev_c < abs_da)
          {
            off_c(-offset_c);
            at(offset, z_mask);
            off_c(offset_c);
          }
          else
          {
            off_a(-offset_a);
            at(offset, z_mask);
            off_a(offset_a);
          }
        }
        else
        {
          off_c(offset_c);
          error_c -= abs_da;

          if (error_c + errorprev_c < abs_da)
          {
            off_c(-offset_c);
            at(offset, z_mask);
            off_c(offset_c);
          }
          else
          {
            off_a(-offset_a);
            at(offset, z_mask);
            off_a(offset_a);
          }


          off_b(offset_b);
          error_b -= abs_da;

          if (error_b + errorprev_b < abs_da)
          {
            off_b(-offset_b);
            at(offset, z_mask);
            off_b(offset_b);
          }
          else
          {
            off_a(-offset_a);
            at(offset, z_mask);
            off_a(offset_a);
          }
        }
      }

      if ((unsigned int)error_b >= abs_da)
      {
        off_b(offset_b);
        error_b -= abs_da;

        if (error_b + errorprev_b < abs_da)
        {
          off_b(-offset_b);
          at(offset, z_mask);
          off_b(offset_b);
        }
        else
        {
          off_a(-offset_a);
          at(offset, z_mask);
          off_a(offset_a);
        }
        // TODO(ahoy): can add both case here
      }
      if ((unsigned int)error_c >= abs_da)
      {
        off_c(offset_c);
        error_c -= abs_da;

        if (error_c + errorprev_c < abs_da)
        {
          off_c(-offset_c);
          at(offset, z_mask);
          off_c(offset_c);
        }
        else
        {
          off_a(-offset_a);
          at(offset, z_mask);
          off_a(offset_a);
        }
        // TODO(ahoy): can add both case here
      }

      errorprev_b = error_b;
      errorprev_c = error_c;
    }
    at(offset, z_mask);
  }

  inline int sign(int i)
  {
    return i > 0 ? 1 : -1;
  }

  inline unsigned int max(unsigned int x, unsigned int y)
  {
    return x > y ? x : y;
  }

  unsigned int size_x_, size_y_, size_z_;
  VoxelDataColumn *data_;
  unsigned char *costmap;

  //Aren't functors so much fun... used to recreate the Bresenham macro Eric wrote in the original version, but in "proper" c++
  class MarkVoxel
  {
  public:
    MarkVoxel(VoxelDataColumn* data): data_(data){}
    inline void operator()(unsigned int offset, VoxelDataColumn z_mask)
    {
      data_[offset] |= z_mask; //clear unknown and mark cell
    }
  private:
    VoxelDataColumn* data_;
  };

  class ClearVoxel
  {
  public:
    ClearVoxel(VoxelDataColumn* data): data_(data){}
    inline void operator()(unsigned int offset, VoxelDataColumn z_mask)
    {
      data_[offset] &= ~(z_mask); //clear unknown and clear cell
    }
  private:
    VoxelDataColumn* data_;
  };

  class ClearVoxelInMap
  {
  public:
    ClearVoxelInMap(
      VoxelDataColumn* data, unsigned char *costmap,
      unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold,
      unsigned char free_cost = 0, unsigned char unknown_cost = 255): data_(data), costmap_(costmap),
      unknown_clear_threshold_(unknown_clear_threshold), marked_clear_threshold_(marked_clear_threshold),
      free_cost_(free_cost), unknown_cost_(unknown_cost)
    {
    }

    inline void operator()(unsigned int offset, VoxelDataColumn z_mask)
    {
      VoxelDataColumn* col = &data_[offset];
      *col &= ~(z_mask); //clear unknown and clear cell

      VoxelMarkedColumn unknown_bits = VoxelMarkedColumn(*col>>Z_VOXELS) ^ VoxelMarkedColumn(*col);
      VoxelMarkedColumn marked_bits = *col>>Z_VOXELS;

      //make sure the number of bits in each is below our thesholds
      if (bitsBelowThreshold(marked_bits, marked_clear_threshold_))
      {
        if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_))
        {
          costmap_[offset] = free_cost_;
        }
        else
        {
          costmap_[offset] = unknown_cost_;
        }
      }
    }
  private:
    inline bool bitsBelowThreshold(VoxelDataColumn n, unsigned int bit_threshold)
    {
      unsigned int bit_count;
      for (bit_count = 0; n;)
      {
        ++bit_count;
        if (bit_count > bit_threshold)
        {
          return false;
        }
        n &= n - 1LL; //clear the least significant bit set
      }
      return true;
    }

    VoxelDataColumn* data_;
    unsigned char *costmap_;
    unsigned int unknown_clear_threshold_, marked_clear_threshold_;
    unsigned char free_cost_, unknown_cost_;
  };

  class GridOffset
  {
  public:
    GridOffset(unsigned int &offset) : offset_(offset) {}
    inline void operator()(int offset_val)
    {
      offset_ += offset_val;
    }
  private:
    unsigned int &offset_;
  };

  class ZOffset
  {
  public:
    ZOffset(VoxelDataColumn &z_mask) : z_mask_(z_mask) {}
    inline void operator()(int offset_val)
    {
      offset_val > 0 ? z_mask_ <<= 1LL : z_mask_ >>= 1LL;
    }
  private:
    VoxelDataColumn & z_mask_;
  };
};

}  // namespace voxel_grid

#endif  // VOXEL_GRID_VOXEL_GRID_H
