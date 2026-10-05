#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

/**
 * @brief 3D Euclidean Distance Transform (EDT) implementation
 *
 * Based on Felzenszwalb & Huttenlocher's efficient EDT algorithm.
 * Computes the squared Euclidean distance from each grid cell to the nearest obstacle.
 *
 * The algorithm works in 3 passes (one per dimension):
 * 1. Process all Z columns
 * 2. Process all Y rows
 * 3. Process all X rows
 *
 * Time complexity: O(N) where N is total number of voxels
 * Space complexity: O(N)
 */
class EDT3D {
public:
    /**
     * @brief Compute 3D Euclidean Distance Transform
     * @param occupancy Binary occupancy grid (1=free, 0=obstacle)
     * @param nx, ny, nz Grid dimensions
     * @param voxel_res Voxel resolution in meters
     * @param distance_field Output distance field (in meters)
     */
    static void compute(const std::vector<uint8_t>& occupancy,
                       int nx, int ny, int nz,
                       double voxel_res,
                       std::vector<float>& distance_field) {

        int total = nx * ny * nz;
        distance_field.resize(total);

        // Initialize: obstacles get 0, free space gets inf
        const float INF = std::numeric_limits<float>::max();
        for (int i = 0; i < total; ++i) {
            distance_field[i] = occupancy[i] ? INF : 0.0f;
        }

        // Temporary storage for squared distances (in voxel units)
        std::vector<float> temp(total);

        // Pass 1: Process along Z axis
        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                // Extract column
                std::vector<float> column(nz);
                for (int z = 0; z < nz; ++z) {
                    column[z] = distance_field[index(x, y, z, nx, ny)];
                }

                // 1D EDT
                std::vector<float> result(nz);
                edt1d(column, result);

                // Write back
                for (int z = 0; z < nz; ++z) {
                    temp[index(x, y, z, nx, ny)] = result[z];
                }
            }
        }

        // Pass 2: Process along Y axis
        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                // Extract row
                std::vector<float> row(ny);
                for (int y = 0; y < ny; ++y) {
                    row[y] = temp[index(x, y, z, nx, ny)];
                }

                // 1D EDT
                std::vector<float> result(ny);
                edt1d(row, result);

                // Write back
                for (int y = 0; y < ny; ++y) {
                    distance_field[index(x, y, z, nx, ny)] = result[y];
                }
            }
        }

        // Pass 3: Process along X axis
        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                // Extract row
                std::vector<float> row(nx);
                for (int x = 0; x < nx; ++x) {
                    row[x] = distance_field[index(x, y, z, nx, ny)];
                }

                // 1D EDT
                std::vector<float> result(nx);
                edt1d(row, result);

                // Write back
                for (int x = 0; x < nx; ++x) {
                    temp[index(x, y, z, nx, ny)] = result[x];
                }
            }
        }

        // Convert squared distances to actual distances in meters
        for (int i = 0; i < total; ++i) {
            if (temp[i] < INF * 0.9f) {
                distance_field[i] = std::sqrt(temp[i]) * voxel_res;
            } else {
                distance_field[i] = INF;  // Keep as inf for unreachable cells
            }
        }
    }

private:
    /**
     * @brief Compute 1D squared Euclidean distance transform
     * @param input Input distances (squared)
     * @param output Output distances (squared)
     *
     * Uses the parabola envelope algorithm for O(n) complexity
     */
    static void edt1d(const std::vector<float>& input, std::vector<float>& output) {
        int n = input.size();
        output.resize(n);

        // Trivial cases
        if (n == 0) return;
        if (n == 1) {
            output[0] = input[0];
            return;
        }

        const float INF = std::numeric_limits<float>::max();
        const auto first_finite = std::find_if(
            input.begin(), input.end(), [INF](float value) {
                return value < INF * 0.9f;
            });
        if (first_finite == input.end()) {
            std::fill(output.begin(), output.end(), INF);
            return;
        }

        std::vector<int> v(n);      // Locations of parabolas in lower envelope
        std::vector<float> z(n + 1); // Locations of boundaries between parabolas
        int k = 0;                   // Index of rightmost parabola

        const int first = static_cast<int>(first_finite - input.begin());
        v[0] = first;
        z[0] = -std::numeric_limits<float>::max();
        z[1] = std::numeric_limits<float>::max();

        // Forward pass: build lower envelope
        for (int q = first + 1; q < n; ++q) {
            if (input[q] >= INF * 0.9f) {
                // Skip infinite values in forward pass
                continue;
            }

            // Find intersection point
            float s = 0.0f;
            while (true) {
                int vk = v[k];
                s = ((input[q] + q * q) - (input[vk] + vk * vk)) / (2.0f * q - 2.0f * vk);
                if (s > z[k] || k == 0) {
                    break;
                }
                --k;
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<float>::max();
        }

        // Backward pass: fill output using lower envelope
        k = 0;
        for (int q = 0; q < n; ++q) {
            while (z[k + 1] < q) {
                ++k;
            }

            int vk = v[k];
            if (input[vk] >= INF * 0.9f) {
                output[q] = INF;
            } else {
                float dq = q - vk;
                output[q] = dq * dq + input[vk];
            }
        }
    }

    /**
     * @brief Convert 3D coordinates to 1D array index
     */
    static inline int index(int x, int y, int z, int nx, int ny) {
        return (z * ny + y) * nx + x;
    }
};

