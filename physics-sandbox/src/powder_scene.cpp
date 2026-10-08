#include "powder_scene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace {

std::mt19937& PowderRng() {
    static std::mt19937 generator(0xC0FFEEu);
    return generator;
}

bool RandomChance(float probability) {
    std::bernoulli_distribution distribution(std::clamp(probability, 0.0f, 1.0f));
    return distribution(PowderRng());
}

int RandomSign() {
    return std::uniform_int_distribution<int>(0, 1)(PowderRng()) == 0 ? -1 : 1;
}

} // namespace

PowderScene::PowderScene()
    : cells_(static_cast<std::size_t>(kGridWidth * kGridHeight)),
      updatedStamp_(static_cast<std::size_t>(kGridWidth * kGridHeight), 0),
      volumeCells_(static_cast<std::size_t>(kVolumeWidth * kVolumeHeight * kVolumeDepth)),
      volumeUpdatedStamp_(static_cast<std::size_t>(kVolumeWidth * kVolumeHeight * kVolumeDepth), 0) {
    Reset();
}

void PowderScene::Reset() {
    std::fill(cells_.begin(), cells_.end(), Cell{});
    std::fill(updatedStamp_.begin(), updatedStamp_.end(), 0);
    stepStamp_ = 0;
    volumeStepStamp_ = 0;
    updateAccumulator_ = 0.0f;
    selectedMaterial_ = Material::Sand;
    SeedDemo();
    SeedVolumeDemo();
    Recount();
    RecountVolume();
}

void PowderScene::SeedDemo() {
    // A small starter scene: sand above a stone floor, plus a water pocket and wood.
    for (int x = 0; x < kGridWidth; ++x) SetCell(x, 0, Material::Stone);
    for (int y = 0; y < 20; ++y) {
        SetCell(1, y, Material::Stone);
        SetCell(kGridWidth - 2, y, Material::Stone);
    }
    for (int y = 74; y < 91; ++y) {
        for (int x = 42; x < 88; ++x) {
            if (std::abs(x - 65) < 22 - (y - 74) / 2) SetCell(x, y, Material::Sand);
        }
    }
    for (int y = 8; y < 26; ++y) {
        for (int x = 103; x < 135; ++x) SetCell(x, y, Material::Water);
    }
    for (int x = 112; x < 128; ++x) SetCell(x, 26, Material::Wood);
    for (int x = 116; x < 124; ++x) SetCell(x, 27, Material::Wood);
}

void PowderScene::SeedVolumeDemo() {
    std::fill(volumeCells_.begin(), volumeCells_.end(), Cell{});
    std::fill(volumeUpdatedStamp_.begin(), volumeUpdatedStamp_.end(), 0);
    for (int z = 0; z < kVolumeDepth; ++z) {
        for (int x = 0; x < kVolumeWidth; ++x) SetVolumeCell(x, 0, z, Material::Stone);
    }
    for (int y = 0; y < kVolumeHeight; ++y) {
        for (int z = 0; z < kVolumeDepth; ++z) {
            SetVolumeCell(0, y, z, Material::Stone);
            SetVolumeCell(kVolumeWidth - 1, y, z, Material::Stone);
        }
        for (int x = 0; x < kVolumeWidth; ++x) {
            SetVolumeCell(x, y, 0, Material::Stone);
            SetVolumeCell(x, y, kVolumeDepth - 1, Material::Stone);
        }
    }

    for (int y = 30; y < 42; ++y) {
        for (int x = 15; x < 43; ++x) {
            for (int z = 8; z < 24; ++z) {
                const float dx = static_cast<float>(x - 29) / 1.0f;
                const float dz = static_cast<float>(z - 16) / 0.8f;
                if (dx * dx + dz * dz < static_cast<float>((42 - y) * (42 - y))) SetVolumeCell(x, y, z, Material::Sand);
            }
        }
    }
    for (int y = 5; y < 16; ++y) {
        for (int x = 43; x < 57; ++x) {
            for (int z = 9; z < 23; ++z) SetVolumeCell(x, y, z, Material::Water);
        }
    }
    for (int x = 46; x < 55; ++x) {
        for (int z = 11; z < 21; ++z) SetVolumeCell(x, 16, z, Material::Wood);
    }
}

void PowderScene::Step(float dt) {
    updateAccumulator_ += std::clamp(dt, 0.0f, 0.1f);
    while (updateAccumulator_ >= 1.0f / 60.0f) {
        updateAccumulator_ -= 1.0f / 60.0f;
        if (threeDimensional_) UpdateGrid3D();
        else UpdateGrid();
    }
}

void PowderScene::UpdateGrid() {
    ++stepStamp_;
    if (stepStamp_ == 0) {
        std::fill(updatedStamp_.begin(), updatedStamp_.end(), 0);
        stepStamp_ = 1;
    }

    // Traverse bottom-to-top so falling grains do not move twice in one tick.
    for (int y = 1; y < kGridHeight; ++y) {
        const bool reverse = ((y + static_cast<int>(stepStamp_)) & 1) != 0;
        if (!reverse) {
            for (int x = 1; x < kGridWidth - 1; ++x) UpdateCell(x, y);
        } else {
            for (int x = kGridWidth - 2; x >= 1; --x) UpdateCell(x, y);
        }
    }

    Recount();
}

void PowderScene::UpdateGrid3D() {
    ++volumeStepStamp_;
    if (volumeStepStamp_ == 0) {
        std::fill(volumeUpdatedStamp_.begin(), volumeUpdatedStamp_.end(), 0);
        volumeStepStamp_ = 1;
    }

    for (int y = 1; y < kVolumeHeight; ++y) {
        const bool reverse = ((y + static_cast<int>(volumeStepStamp_)) & 1) != 0;
        for (int z = 1; z < kVolumeDepth - 1; ++z) {
            if (!reverse) {
                for (int x = 1; x < kVolumeWidth - 1; ++x) UpdateCell3D(x, y, z);
            } else {
                for (int x = kVolumeWidth - 2; x >= 1; --x) UpdateCell3D(x, y, z);
            }
        }
    }
    RecountVolume();
}

void PowderScene::UpdateCell3D(int x, int y, int z) {
    if (!VolumeInBounds(x, y, z) || volumeUpdatedStamp_[VolumeIndex(x, y, z)] == volumeStepStamp_) return;
    Cell& cell = volumeCells_[VolumeIndex(x, y, z)];
    if (cell.material == Material::Empty) return;
    volumeUpdatedStamp_[VolumeIndex(x, y, z)] = volumeStepStamp_;
    if (cell.temperature > 20.0f) cell.temperature = std::max(20.0f, cell.temperature - 0.35f);

    const int directionX = RandomSign();
    const int directionZ = RandomSign();
    const int diagonalX = x + directionX;
    const int diagonalZ = z + directionZ;
    switch (cell.material) {
        case Material::Sand:
            if (Move3D(x, y, z, x, y - 1, z, true)) return;
            if (Move3D(x, y, z, diagonalX, y - 1, z, true)) return;
            if (Move3D(x, y, z, x, y - 1, diagonalZ, true)) return;
            if (Move3D(x, y, z, diagonalX, y - 1, diagonalZ, true)) return;
            break;
        case Material::Water:
        case Material::Oil:
            if (Move3D(x, y, z, x, y - 1, z, true)) return;
            if (Move3D(x, y, z, diagonalX, y - 1, z, true)) return;
            if (Move3D(x, y, z, x, y - 1, diagonalZ, true)) return;
            if (Move3D(x, y, z, diagonalX, y - 1, diagonalZ, true)) return;
            if (RandomChance(0.7f) && Move3D(x, y, z, x + directionX, y, z, true)) return;
            Move3D(x, y, z, x, y, z + directionZ, true);
            break;
        case Material::Smoke:
        case Material::Steam:
            if (cell.life > 0) --cell.life;
            if (cell.life == 0) {
                SetVolumeCell(x, y, z, Material::Empty);
                return;
            }
            if (Move3D(x, y, z, x, y + 1, z, false)) return;
            if (Move3D(x, y, z, diagonalX, y + 1, z, false)) return;
            if (Move3D(x, y, z, x, y + 1, diagonalZ, false)) return;
            Move3D(x, y, z, diagonalX, y + 1, diagonalZ, false);
            break;
        case Material::Fire:
            if (cell.life > 0) --cell.life;
            cell.temperature = std::max(cell.temperature, 500.0f);
            IgniteNeighbors3D(x, y, z);
            if (cell.life == 0) {
                SetVolumeCell(x, y, z, Material::Smoke, 80, 180.0f);
                return;
            }
            if (Move3D(x, y, z, x, y + 1, z, false)) return;
            if (Move3D(x, y, z, diagonalX, y + 1, z, false)) return;
            if (Move3D(x, y, z, x, y + 1, diagonalZ, false)) return;
            Move3D(x, y, z, diagonalX, y + 1, diagonalZ, false);
            break;
        case Material::Acid:
            if (Move3D(x, y, z, x, y - 1, z, true)) return;
            if (Move3D(x, y, z, diagonalX, y - 1, z, true)) return;
            if (Move3D(x, y, z, x, y - 1, diagonalZ, true)) return;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = x + dx;
                    const int nz = z + dz;
                    if (!VolumeInBounds(nx, y, nz)) continue;
                    const Material neighbor = volumeCells_[VolumeIndex(nx, y, nz)].material;
                    if ((neighbor == Material::Wood || neighbor == Material::Stone) && RandomChance(0.01f)) SetVolumeCell(nx, y, nz, Material::Smoke, 55, 110.0f);
                }
            }
            break;
        case Material::Empty:
        case Material::Stone:
        case Material::Wood:
        case Material::Count:
            break;
    }
}

bool PowderScene::Move3D(int x, int y, int z, int nx, int ny, int nz, bool allowSwap) {
    if (!VolumeInBounds(nx, ny, nz) || !VolumeInBounds(x, y, z) || volumeUpdatedStamp_[VolumeIndex(nx, ny, nz)] == volumeStepStamp_) return false;
    Cell& source = volumeCells_[VolumeIndex(x, y, z)];
    Cell& destination = volumeCells_[VolumeIndex(nx, ny, nz)];
    if (destination.material == Material::Empty) {
        destination = source;
        source = Cell{};
        volumeUpdatedStamp_[VolumeIndex(nx, ny, nz)] = volumeStepStamp_;
        return true;
    }
    if (!allowSwap || destination.material == Material::Stone || destination.material == Material::Wood || destination.material == Material::Fire || destination.material == Material::Smoke || destination.material == Material::Steam) return false;
    if (Density(source.material) <= Density(destination.material)) return false;
    std::swap(source, destination);
    volumeUpdatedStamp_[VolumeIndex(nx, ny, nz)] = volumeStepStamp_;
    volumeUpdatedStamp_[VolumeIndex(x, y, z)] = volumeStepStamp_;
    return true;
}

void PowderScene::UpdateCell(int x, int y) {
    if (!InBounds(x, y) || updatedStamp_[Index(x, y)] == stepStamp_) return;
    Cell& cell = cells_[Index(x, y)];
    if (cell.material == Material::Empty) return;
    updatedStamp_[Index(x, y)] = stepStamp_;

    if (cell.temperature > 20.0f) cell.temperature = std::max(20.0f, cell.temperature - 0.35f);

    const int direction = RandomSign();
    switch (cell.material) {
        case Material::Sand:
            if (Move(x, y, x, y - 1, true)) return;
            if (Move(x, y, x + direction, y - 1, true)) return;
            Move(x, y, x - direction, y - 1, true);
            break;
        case Material::Water:
        case Material::Oil:
            if (Move(x, y, x, y - 1, true)) return;
            if (Move(x, y, x + direction, y - 1, true)) return;
            if (Move(x, y, x - direction, y - 1, true)) return;
            if (RandomChance(0.7f) && Move(x, y, x + direction, y, true)) return;
            Move(x, y, x - direction, y, true);
            break;
        case Material::Smoke:
        case Material::Steam:
            if (cell.life > 0) --cell.life;
            if (cell.life == 0) {
                SetCell(x, y, Material::Empty);
                return;
            }
            if (Move(x, y, x, y + 1, false)) return;
            if (Move(x, y, x + direction, y + 1, false)) return;
            Move(x, y, x - direction, y + 1, false);
            break;
        case Material::Fire:
            if (cell.life > 0) --cell.life;
            cell.temperature = std::max(cell.temperature, 500.0f);
            IgniteNeighbors(x, y);
            if (cell.life == 0) {
                SetCell(x, y, Material::Smoke, 80, 180.0f);
                return;
            }
            if (Move(x, y, x, y + 1, false)) return;
            if (Move(x, y, x + direction, y + 1, false)) return;
            Move(x, y, x - direction, y + 1, false);
            break;
        case Material::Acid:
            if (Move(x, y, x, y - 1, true)) return;
            if (Move(x, y, x + direction, y - 1, true)) return;
            if (Move(x, y, x - direction, y - 1, true)) return;
            for (const auto offset : std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
                const int nx = x + offset.first;
                const int ny = y + offset.second;
                if (!InBounds(nx, ny)) continue;
                const Material neighbor = cells_[Index(nx, ny)].material;
                if ((neighbor == Material::Wood || neighbor == Material::Stone) && RandomChance(0.015f)) {
                    SetCell(nx, ny, Material::Smoke, 55, 110.0f);
                }
            }
            break;
        case Material::Empty:
        case Material::Stone:
        case Material::Wood:
        case Material::Count:
            break;
    }
}

bool PowderScene::Move(int x, int y, int nx, int ny, bool allowSwap) {
    if (!InBounds(nx, ny) || !InBounds(x, y) || updatedStamp_[Index(nx, ny)] == stepStamp_) return false;
    Cell& source = cells_[Index(x, y)];
    Cell& destination = cells_[Index(nx, ny)];
    if (destination.material == Material::Empty) {
        destination = source;
        source = Cell{};
        updatedStamp_[Index(nx, ny)] = stepStamp_;
        return true;
    }
    if (!allowSwap || destination.material == Material::Stone || destination.material == Material::Wood || destination.material == Material::Fire || destination.material == Material::Smoke || destination.material == Material::Steam) return false;
    if (Density(source.material) <= Density(destination.material)) return false;
    std::swap(source, destination);
    updatedStamp_[Index(nx, ny)] = stepStamp_;
    updatedStamp_[Index(x, y)] = stepStamp_;
    return true;
}

bool PowderScene::IsEmpty(int x, int y) const { return InBounds(x, y) && cells_[Index(x, y)].material == Material::Empty; }
bool PowderScene::IsEmpty3D(int x, int y, int z) const { return VolumeInBounds(x, y, z) && volumeCells_[VolumeIndex(x, y, z)].material == Material::Empty; }
bool PowderScene::IsLiquid(Material material) const { return material == Material::Water || material == Material::Oil || material == Material::Acid; }
bool PowderScene::IsGas(Material material) const { return material == Material::Fire || material == Material::Smoke || material == Material::Steam; }
bool PowderScene::IsFlammable(Material material) const { return material == Material::Wood || material == Material::Oil; }

float PowderScene::Density(Material material) const {
    switch (material) {
        case Material::Sand: return 2.4f;
        case Material::Stone: return 4.0f;
        case Material::Water: return 1.0f;
        case Material::Oil: return 0.75f;
        case Material::Acid: return 1.1f;
        case Material::Wood: return 0.6f;
        default: return 0.05f;
    }
}

void PowderScene::IgniteNeighbors(int x, int y) {
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int nx = x + dx;
            const int ny = y + dy;
            if (!InBounds(nx, ny)) continue;
            Cell& neighbor = cells_[Index(nx, ny)];
            if (IsFlammable(neighbor.material) && RandomChance(neighbor.material == Material::Oil ? 0.18f : 0.05f)) {
                SetCell(nx, ny, Material::Fire, 45, 700.0f);
            }
            if (neighbor.material == Material::Water && RandomChance(0.03f)) {
                SetCell(x, y, Material::Steam, 75, 120.0f);
            }
        }
    }
}

void PowderScene::IgniteNeighbors3D(int x, int y, int z) {
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                const int nx = x + dx;
                const int ny = y + dy;
                const int nz = z + dz;
                if (!VolumeInBounds(nx, ny, nz)) continue;
                Cell& neighbor = volumeCells_[VolumeIndex(nx, ny, nz)];
                if (IsFlammable(neighbor.material) && RandomChance(neighbor.material == Material::Oil ? 0.14f : 0.04f)) SetVolumeCell(nx, ny, nz, Material::Fire, 45, 700.0f);
                if (neighbor.material == Material::Water && RandomChance(0.02f)) SetVolumeCell(x, y, z, Material::Steam, 75, 120.0f);
            }
        }
    }
}

void PowderScene::SetCell(int x, int y, Material material, std::uint16_t life, float temperature) {
    if (!InBounds(x, y)) return;
    cells_[Index(x, y)] = Cell{material, life, temperature};
}

void PowderScene::SetVolumeCell(int x, int y, int z, Material material, std::uint16_t life, float temperature) {
    if (!VolumeInBounds(x, y, z)) return;
    volumeCells_[VolumeIndex(x, y, z)] = Cell{material, life, temperature};
}

void PowderScene::Paint(int x, int y, int radius, Material material) {
    radius = std::clamp(radius, 1, 12);
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius || !InBounds(x + dx, y + dy)) continue;
            const int px = x + dx;
            const int py = y + dy;
            if (material == Material::Empty || cells_[Index(px, py)].material == Material::Empty || material == Material::Fire || material == Material::Water || material == Material::Oil || material == Material::Sand || material == Material::Acid) {
                const std::uint16_t life = material == Material::Fire ? 45 : (IsGas(material) ? 80 : 0);
                SetCell(px, py, material, life, material == Material::Fire ? 700.0f : 20.0f);
            }
        }
    }
    Recount();
}

void PowderScene::PaintVolume(int x, int y, int z, int radius, Material material) {
    radius = std::clamp(radius, 1, 8);
    for (int dz = -radius; dz <= radius; ++dz) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy + dz * dz > radius * radius || !VolumeInBounds(x + dx, y + dy, z + dz)) continue;
                const int px = x + dx;
                const int py = y + dy;
                const int pz = z + dz;
                Cell& target = volumeCells_[VolumeIndex(px, py, pz)];
                if (material == Material::Empty || target.material == Material::Empty || material == Material::Fire || material == Material::Water || material == Material::Oil || material == Material::Sand || material == Material::Acid) {
                    const std::uint16_t life = material == Material::Fire ? 45 : (IsGas(material) ? 80 : 0);
                    SetVolumeCell(px, py, pz, material, life, material == Material::Fire ? 700.0f : 20.0f);
                }
            }
        }
    }
    RecountVolume();
}

void PowderScene::Erase(int x, int y, int radius) { Paint(x, y, radius, Material::Empty); }

void PowderScene::EraseVolume(int x, int y, int z, int radius) { PaintVolume(x, y, z, radius, Material::Empty); }

void PowderScene::AddImpulse(int x, int y, int radius) {
    radius = std::clamp(radius, 2, 16);
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius || !InBounds(x + dx, y + dy)) continue;
            Cell& cell = cells_[Index(x + dx, y + dy)];
            if (cell.material == Material::Sand || cell.material == Material::Water || cell.material == Material::Oil || cell.material == Material::Acid) {
                if (RandomChance(0.45f)) {
                    const int nx = std::clamp(x + dx + (dx >= 0 ? 2 : -2), 1, kGridWidth - 2);
                    const int ny = std::clamp(y + dy + 5, 1, kGridHeight - 2);
                    if (IsEmpty(nx, ny)) {
                        cells_[Index(nx, ny)] = cell;
                        cell = Cell{};
                    }
                }
            }
        }
    }
    Recount();
}

void PowderScene::AddVolumeSource() {
    const int centerX = kVolumeWidth / 2;
    const int centerY = kVolumeHeight - 6;
    const int centerZ = kVolumeDepth / 2;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dz = -5; dz <= 5; ++dz) {
            for (int dx = -5; dx <= 5; ++dx) {
                if (dx * dx + dz * dz > 30 || !VolumeInBounds(centerX + dx, centerY + dy, centerZ + dz)) continue;
                const std::uint16_t life = selectedMaterial_ == Material::Fire ? 45 : (IsGas(selectedMaterial_) ? 80 : 0);
                const float temperature = selectedMaterial_ == Material::Fire ? 700.0f : 20.0f;
                SetVolumeCell(centerX + dx, centerY + dy, centerZ + dz, selectedMaterial_, life, temperature);
            }
        }
    }
    RecountVolume();
}

PowderScene::Cell PowderScene::ProjectedCellAt(int x, int y) const {
    if (!threeDimensional_) return InBounds(x, y) ? cells_[Index(x, y)] : Cell{};
    if (!VolumeInBounds(x, y, 0)) return Cell{};
    for (int z = kVolumeDepth - 1; z >= 0; --z) {
        const Cell& cell = volumeCells_[VolumeIndex(x, y, z)];
        if (cell.material != Material::Empty) return cell;
    }
    return Cell{};
}

int PowderScene::Count(Material material) const {
    int count = 0;
    const std::vector<Cell>& activeCells = threeDimensional_ ? volumeCells_ : cells_;
    for (const Cell& cell : activeCells) if (cell.material == material) ++count;
    return count;
}

int PowderScene::OccupiedCount() const {
    return threeDimensional_ ? volumeOccupiedCount_ : occupiedCount_;
}

void PowderScene::CycleMaterial(int direction) {
    int value = static_cast<int>(selectedMaterial_);
    value = (value + direction) % static_cast<int>(Material::Count);
    if (value < 0) value += static_cast<int>(Material::Count);
    if (value == static_cast<int>(Material::Empty)) value = direction > 0 ? 1 : static_cast<int>(Material::Count) - 1;
    selectedMaterial_ = static_cast<Material>(value);
}

const char* PowderScene::SelectedMaterialName() const { return MaterialName(selectedMaterial_); }

const char* PowderScene::MaterialName(Material material) {
    switch (material) {
        case Material::Sand: return "SAND";
        case Material::Water: return "WATER";
        case Material::Stone: return "STONE";
        case Material::Wood: return "WOOD";
        case Material::Fire: return "FIRE";
        case Material::Smoke: return "SMOKE";
        case Material::Steam: return "STEAM";
        case Material::Oil: return "OIL";
        case Material::Acid: return "ACID";
        case Material::Empty: return "EMPTY";
        case Material::Count: break;
    }
    return "UNKNOWN";
}

Color PowderScene::MaterialColor(Material material, float temperature) {
    if (material == Material::Fire) {
        const float heat = std::clamp((temperature - 400.0f) / 500.0f, 0.0f, 1.0f);
        return {255, static_cast<unsigned char>(90.0f + 130.0f * (1.0f - heat)), static_cast<unsigned char>(35.0f + 45.0f * (1.0f - heat)), 255};
    }
    switch (material) {
        case Material::Sand: return {224, 185, 103, 255};
        case Material::Water: return {58, 148, 224, 230};
        case Material::Stone: return {112, 124, 141, 255};
        case Material::Wood: return {154, 91, 48, 255};
        case Material::Smoke: return {118, 124, 140, 190};
        case Material::Steam: return {180, 220, 255, 175};
        case Material::Oil: return {84, 73, 45, 245};
        case Material::Acid: return {104, 224, 119, 245};
        default: return {0, 0, 0, 0};
    }
}

void PowderScene::Recount() {
    occupiedCount_ = 0;
    for (const Cell& cell : cells_) if (cell.material != Material::Empty) ++occupiedCount_;
}

void PowderScene::RecountVolume() {
    volumeOccupiedCount_ = 0;
    for (const Cell& cell : volumeCells_) if (cell.material != Material::Empty) ++volumeOccupiedCount_;
}
