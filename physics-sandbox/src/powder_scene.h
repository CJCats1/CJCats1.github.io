#pragma once

#include "raylib.h"

#include <cstdint>
#include <vector>

class PowderScene {
public:
    static constexpr int kGridWidth = 160;
    static constexpr int kGridHeight = 100;
    static constexpr float kCellSize = 0.06f;
    static constexpr int kVolumeWidth = 64;
    static constexpr int kVolumeHeight = 48;
    static constexpr int kVolumeDepth = 32;
    static constexpr float kVolumeCellSize = 0.16f;

    enum class Material : std::uint8_t {
        Empty = 0,
        Sand,
        Water,
        Stone,
        Wood,
        Fire,
        Smoke,
        Steam,
        Oil,
        Acid,
        Count
    };

    struct Cell {
        Material material = Material::Empty;
        std::uint16_t life = 0;
        float temperature = 20.0f;
    };

    PowderScene();

    void Reset();
    void Step(float dt);
    void SetThreeDimensional(bool enabled) { threeDimensional_ = enabled; }
    bool IsThreeDimensional() const { return threeDimensional_; }
    void Paint(int x, int y, int radius, Material material);
    void PaintVolume(int x, int y, int z, int radius, Material material);
    void Erase(int x, int y, int radius);
    void EraseVolume(int x, int y, int z, int radius);
    void AddImpulse(int x, int y, int radius);
    void AddVolumeSource();

    const std::vector<Cell>& Cells() const { return cells_; }
    const Cell& CellAt(int x, int y) const { return cells_[Index(x, y)]; }
    const Cell& VolumeCellAt(int x, int y, int z) const { return volumeCells_[VolumeIndex(x, y, z)]; }
    Cell ProjectedCellAt(int x, int y) const;
    int Count(Material material) const;
    int OccupiedCount() const;
    int ActiveGridWidth() const { return threeDimensional_ ? kVolumeWidth : kGridWidth; }
    int ActiveGridHeight() const { return threeDimensional_ ? kVolumeHeight : kGridHeight; }
    int ActiveGridDepth() const { return threeDimensional_ ? kVolumeDepth : 1; }
    float ActiveCellSize() const { return threeDimensional_ ? kVolumeCellSize : kCellSize; }
    Material SelectedMaterial() const { return selectedMaterial_; }
    void SetSelectedMaterial(Material material) { selectedMaterial_ = material; }
    void CycleMaterial(int direction);
    const char* SelectedMaterialName() const;
    static const char* MaterialName(Material material);
    static Color MaterialColor(Material material, float temperature = 20.0f);

private:
    static constexpr int Index(int x, int y) { return y * kGridWidth + x; }
    static bool InBounds(int x, int y) { return x >= 0 && x < kGridWidth && y >= 0 && y < kGridHeight; }
    static constexpr int VolumeIndex(int x, int y, int z) { return x + kVolumeWidth * (y + kVolumeHeight * z); }
    static bool VolumeInBounds(int x, int y, int z) { return x >= 0 && x < kVolumeWidth && y >= 0 && y < kVolumeHeight && z >= 0 && z < kVolumeDepth; }

    void SeedDemo();
    void SeedVolumeDemo();
    void UpdateGrid();
    void UpdateGrid3D();
    void UpdateCell(int x, int y);
    void UpdateCell3D(int x, int y, int z);
    bool Move(int x, int y, int nx, int ny, bool allowSwap);
    bool Move3D(int x, int y, int z, int nx, int ny, int nz, bool allowSwap);
    bool IsEmpty(int x, int y) const;
    bool IsEmpty3D(int x, int y, int z) const;
    bool IsLiquid(Material material) const;
    bool IsGas(Material material) const;
    bool IsFlammable(Material material) const;
    float Density(Material material) const;
    void IgniteNeighbors(int x, int y);
    void IgniteNeighbors3D(int x, int y, int z);
    void SetCell(int x, int y, Material material, std::uint16_t life = 0, float temperature = 20.0f);
    void SetVolumeCell(int x, int y, int z, Material material, std::uint16_t life = 0, float temperature = 20.0f);
    void Recount();
    void RecountVolume();

    std::vector<Cell> cells_;
    std::vector<std::uint32_t> updatedStamp_;
    std::vector<Cell> volumeCells_;
    std::vector<std::uint32_t> volumeUpdatedStamp_;
    std::uint32_t stepStamp_ = 0;
    std::uint32_t volumeStepStamp_ = 0;
    float updateAccumulator_ = 0.0f;
    int occupiedCount_ = 0;
    int volumeOccupiedCount_ = 0;
    bool threeDimensional_ = false;
    Material selectedMaterial_ = Material::Sand;
};
