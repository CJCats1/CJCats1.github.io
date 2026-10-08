#include "beam_bending_scene.h"
#include "web_mouse.h"

#include "rlgl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
constexpr double kNewmarkBeta = 0.25;
constexpr double kNewmarkGamma = 0.5;
constexpr double kRayleighAlpha = 0.018;
constexpr double kRayleighBeta = 0.00008;
constexpr float kBeamRadius = 0.045f;
constexpr float kMouseAttachmentStiffness = 18.0f;

struct LocalMatrices {
    std::array<double, 16> stiffness{};
    std::array<double, 16> mass{};
};

LocalMatrices ElementMatrices(double ei, double rhoA, double length) {
    const double l = length;
    const double l2 = l * l;
    const double l3 = l2 * l;
    LocalMatrices matrices;
    matrices.stiffness = {
        12.0, 6.0 * l, -12.0, 6.0 * l,
        6.0 * l, 4.0 * l2, -6.0 * l, 2.0 * l2,
        -12.0, -6.0 * l, 12.0, -6.0 * l,
        6.0 * l, 2.0 * l2, -6.0 * l, 4.0 * l2
    };
    matrices.mass = {
        156.0, 22.0 * l, 54.0, -13.0 * l,
        22.0 * l, 4.0 * l2, 13.0 * l, -3.0 * l2,
        54.0, 13.0 * l, 156.0, -22.0 * l,
        -13.0 * l, -3.0 * l2, -22.0 * l, 4.0 * l2
    };
    for (double& value : matrices.stiffness) value *= ei / l3;
    for (double& value : matrices.mass) value *= rhoA * l / 420.0;
    return matrices;
}

}

double BeamBendingScene::MatrixAt(const std::vector<double>& matrix, int size, int row, int column) { return matrix[static_cast<std::size_t>(row * size + column)]; }
double& BeamBendingScene::MatrixAt(std::vector<double>& matrix, int size, int row, int column) { return matrix[static_cast<std::size_t>(row * size + column)]; }
Vector3 BeamBendingScene::Add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 BeamBendingScene::Subtract(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 BeamBendingScene::Scale(Vector3 value, float scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
float BeamBendingScene::Dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float BeamBendingScene::Length(Vector3 value) { return std::sqrt(Dot(value, value)); }

BeamBendingScene::BeamBendingScene() { Reset(); }

void BeamBendingScene::AddBeam(const char* name, Kind kind, int station, Vector3 origin, Support support,
                               Material material, Section section, double load, double loadPosition,
                               bool dynamic, double tipMass) {
    Beam beam;
    beam.name = name;
    beam.kind = kind;
    beam.station = station;
    beam.origin = origin;
    beam.support = support;
    beam.material = material;
    beam.section = section;
    beam.load = load;
    beam.loadPosition = loadPosition;
    beam.dynamic = dynamic;
    beam.tipMass = tipMass;
    beams_.push_back(beam);
    Assemble(beams_.back());
}

void BeamBendingScene::BuildGallery() {
    beams_.clear();
    const Material polymer{"POLYMER", 2.0e9, 650.0, {126, 183, 255, 255}};
    const Material aluminum{"ALUMINUM", 5.0e9, 780.0, {242, 187, 75, 255}};
    const Material carbon{"CARBON", 1.0e10, 650.0, {152, 112, 231, 255}};
    const Section standard{0.07, 0.045};

    AddBeam("CANTILEVER", Kind::Cantilever, 0, {-5.8f, 3.9f, -1.9f}, Support::Cantilever, polymer, standard, -1.0, 1.0, true);
    AddBeam("DIVING BOARD", Kind::DivingBoard, 1, {-2.8f, 3.9f, -1.9f}, Support::Cantilever, polymer, {0.09, 0.04}, -1.0, 1.0, true, tipMass_);
    AddBeam("MOVING LOAD BRIDGE", Kind::Bridge, 2, {0.2f, 3.9f, -1.9f}, Support::SimplySupported, aluminum, standard, -1.4, 0.5, true);

    AddBeam("POLYMER", Kind::Material, 3, {3.2f, 4.35f, -1.9f}, Support::Cantilever, polymer, standard, -0.9, 1.0, false);
    AddBeam("ALUMINUM", Kind::Material, 3, {3.2f, 3.92f, -1.9f}, Support::Cantilever, aluminum, standard, -0.9, 1.0, false);
    AddBeam("CARBON", Kind::Material, 3, {3.2f, 3.49f, -1.9f}, Support::Cantilever, carbon, standard, -0.9, 1.0, false);

    AddBeam("THIN", Kind::Thickness, 4, {-5.8f, 2.35f, 1.55f}, Support::Cantilever, aluminum, {0.07, 0.027}, -0.8, 1.0, false);
    AddBeam("MEDIUM", Kind::Thickness, 4, {-5.8f, 1.92f, 1.55f}, Support::Cantilever, aluminum, {0.07, 0.045}, -0.8, 1.0, false);
    AddBeam("THICK", Kind::Thickness, 4, {-5.8f, 1.49f, 1.55f}, Support::Cantilever, aluminum, {0.07, 0.065}, -0.8, 1.0, false);

    AddBeam("PERIODIC DRIVE", Kind::Driven, 5, {-2.8f, 2.1f, 1.55f}, Support::Cantilever, carbon, standard, -1.0, 1.0, true);
    AddBeam("CLAMPED-CLAMPED", Kind::Clamped, 6, {0.2f, 2.3f, 1.55f}, Support::ClampedClamped, polymer, standard, -1.1, 0.5, false);
    AddBeam("MODE SHAPE", Kind::ModeShape, 7, {3.2f, 2.35f, 1.55f}, Support::Cantilever, aluminum, standard, 0.0, 0.5, false);
}

void BeamBendingScene::Assemble(Beam& beam) {
    const int nodeCount = beam.elements + 1;
    const int size = nodeCount * 2;
    beam.mass.assign(static_cast<std::size_t>(size * size), 0.0);
    beam.stiffness.assign(static_cast<std::size_t>(size * size), 0.0);
    beam.damping.assign(static_cast<std::size_t>(size * size), 0.0);
    beam.displacement.assign(static_cast<std::size_t>(size), 0.0);
    beam.velocity.assign(static_cast<std::size_t>(size), 0.0);
    beam.acceleration.assign(static_cast<std::size_t>(size), 0.0);
    beam.freeDofs.clear();
    const double elementLength = static_cast<double>(beam.length) / static_cast<double>(beam.elements);
    const LocalMatrices local = ElementMatrices(beam.material.youngsModulus * beam.section.Inertia(), beam.material.density * beam.section.Area(), elementLength);
    for (int element = 0; element < beam.elements; ++element) {
        const int base = element * 2;
        for (int row = 0; row < 4; ++row) for (int column = 0; column < 4; ++column) {
            const int globalRow = base + row;
            const int globalColumn = base + column;
            MatrixAt(beam.stiffness, size, globalRow, globalColumn) += local.stiffness[static_cast<std::size_t>(row * 4 + column)];
            MatrixAt(beam.mass, size, globalRow, globalColumn) += local.mass[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    if (beam.tipMass > 0.0) MatrixAt(beam.mass, size, size - 2, size - 2) += beam.tipMass;
    for (int row = 0; row < size; ++row) for (int column = 0; column < size; ++column) MatrixAt(beam.damping, size, row, column) = kRayleighAlpha * MatrixAt(beam.mass, size, row, column) + kRayleighBeta * MatrixAt(beam.stiffness, size, row, column);
    std::vector<bool> constrained(static_cast<std::size_t>(size), false);
    if (beam.support == Support::Cantilever) { constrained[0] = true; constrained[1] = true; }
    if (beam.support == Support::SimplySupported) { constrained[0] = true; constrained[static_cast<std::size_t>(size - 2)] = true; }
    if (beam.support == Support::ClampedClamped) { constrained[0] = true; constrained[1] = true; constrained[static_cast<std::size_t>(size - 2)] = true; constrained[static_cast<std::size_t>(size - 1)] = true; }
    for (int dof = 0; dof < size; ++dof) if (!constrained[static_cast<std::size_t>(dof)]) beam.freeDofs.push_back(dof);
}

bool BeamBendingScene::SolveLinearSystem(std::vector<double> matrix, std::vector<double> rhs, std::vector<double>& solution) const {
    const int size = static_cast<int>(rhs.size());
    solution.assign(rhs.size(), 0.0);
    for (int pivot = 0; pivot < size; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < size; ++row) if (std::abs(MatrixAt(matrix, size, row, pivot)) > std::abs(MatrixAt(matrix, size, best, pivot))) best = row;
        if (std::abs(MatrixAt(matrix, size, best, pivot)) < 1.0e-12) return false;
        if (best != pivot) { for (int column = pivot; column < size; ++column) std::swap(MatrixAt(matrix, size, pivot, column), MatrixAt(matrix, size, best, column)); std::swap(rhs[static_cast<std::size_t>(pivot)], rhs[static_cast<std::size_t>(best)]); }
        const double diagonal = MatrixAt(matrix, size, pivot, pivot);
        for (int row = pivot + 1; row < size; ++row) {
            const double factor = MatrixAt(matrix, size, row, pivot) / diagonal;
            if (std::abs(factor) < 1.0e-14) continue;
            for (int column = pivot; column < size; ++column) MatrixAt(matrix, size, row, column) -= factor * MatrixAt(matrix, size, pivot, column);
            rhs[static_cast<std::size_t>(row)] -= factor * rhs[static_cast<std::size_t>(pivot)];
        }
    }
    for (int row = size - 1; row >= 0; --row) {
        double value = rhs[static_cast<std::size_t>(row)];
        for (int column = row + 1; column < size; ++column) value -= MatrixAt(matrix, size, row, column) * solution[static_cast<std::size_t>(column)];
        solution[static_cast<std::size_t>(row)] = value / MatrixAt(matrix, size, row, row);
    }
    return true;
}

void BeamBendingScene::AddPointLoad(const Beam& beam, std::vector<double>& force, double position, double value) const {
    const double x = std::clamp(position, 0.0, static_cast<double>(beam.length));
    const double elementLength = static_cast<double>(beam.length) / static_cast<double>(beam.elements);
    const int element = std::min(beam.elements - 1, static_cast<int>(x / elementLength));
    const double local = (x - element * elementLength) / elementLength;
    const double s = std::clamp(local, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const std::array<double, 4> shape = {1.0 - 3.0 * s2 + 2.0 * s3, elementLength * (s - 2.0 * s2 + s3), 3.0 * s2 - 2.0 * s3, elementLength * (-s2 + s3)};
    const int base = element * 2;
    for (int i = 0; i < 4; ++i) force[static_cast<std::size_t>(base + i)] += shape[static_cast<std::size_t>(i)] * value;
}

std::vector<double> BeamBendingScene::ExternalForce(const Beam& beam, double time, bool initial) const {
    const int size = static_cast<int>(beam.displacement.size());
    std::vector<double> force(static_cast<std::size_t>(size), 0.0);
    switch (beam.kind) {
        case Kind::Cantilever: AddPointLoad(beam, force, beam.length, initial ? beam.load : 0.0); break;
        case Kind::DivingBoard: AddPointLoad(beam, force, beam.length, initial ? beam.load * tipMass_ * kGravity : 0.0); break;
        case Kind::Bridge: AddPointLoad(beam, force, beam.length * (0.5 + 0.36 * std::sin(time * 0.7)), beam.load); break;
        case Kind::Material:
        case Kind::Thickness:
        case Kind::Clamped: AddPointLoad(beam, force, beam.length * beam.loadPosition, beam.load); break;
        case Kind::Driven: AddPointLoad(beam, force, beam.length, beam.load * std::sin(2.0 * kPi * driveFrequency_ * time)); break;
        case Kind::ModeShape: break;
    }
    return force;
}

void BeamBendingScene::AddAttachmentForce(const Beam& beam, std::vector<double>& force) const {
    if (!dragging_ || selectedBeam_ < 0 || &beam != &beams_[static_cast<std::size_t>(selectedBeam_)]) return;
    const double current = HermiteDisplacement(beam, dragLocalX_);
    const double target = static_cast<double>(dragTargetWorldY_ - beam.origin.y) / kDisplayScale;
    AddPointLoad(beam, force, dragLocalX_, kMouseAttachmentStiffness * (target - current));
}

void BeamBendingScene::SolveStatic(Beam& beam, const std::vector<double>& force) {
    const int reducedSize = static_cast<int>(beam.freeDofs.size());
    std::vector<double> matrix(static_cast<std::size_t>(reducedSize * reducedSize), 0.0);
    std::vector<double> rhs(static_cast<std::size_t>(reducedSize), 0.0);
    for (int row = 0; row < reducedSize; ++row) {
        rhs[static_cast<std::size_t>(row)] = force[static_cast<std::size_t>(beam.freeDofs[static_cast<std::size_t>(row)])];
        for (int column = 0; column < reducedSize; ++column) MatrixAt(matrix, reducedSize, row, column) = MatrixAt(beam.stiffness, static_cast<int>(beam.displacement.size()), beam.freeDofs[static_cast<std::size_t>(row)], beam.freeDofs[static_cast<std::size_t>(column)]);
    }
    std::vector<double> solution;
    if (!SolveLinearSystem(matrix, rhs, solution)) { solverResidual_ = std::numeric_limits<double>::infinity(); return; }
    std::fill(beam.displacement.begin(), beam.displacement.end(), 0.0);
    for (int i = 0; i < reducedSize; ++i) beam.displacement[static_cast<std::size_t>(beam.freeDofs[static_cast<std::size_t>(i)])] = solution[static_cast<std::size_t>(i)];
}

void BeamBendingScene::SetModeShape(Beam& beam) const {
    const std::array<double, 3> betas = {{1.875104, 4.694091, 7.854757}};
    const double beta = betas[static_cast<std::size_t>(std::clamp(modeNumber_, 1, 3) - 1)];
    const double sigma = (std::cosh(beta) + std::cos(beta)) / (std::sinh(beta) + std::sin(beta));
    std::fill(beam.displacement.begin(), beam.displacement.end(), 0.0);
    for (int node = 0; node <= beam.elements; ++node) {
        const double s = static_cast<double>(node) / static_cast<double>(beam.elements);
        const double shape = std::cosh(beta * s) - std::cos(beta * s) - sigma * (std::sinh(beta * s) - std::sin(beta * s));
        beam.displacement[static_cast<std::size_t>(node * 2)] = 0.22 * shape;
    }
}

void BeamBendingScene::ResetBeam(Beam& beam, bool useInitialLoad) {
    std::fill(beam.velocity.begin(), beam.velocity.end(), 0.0);
    std::fill(beam.acceleration.begin(), beam.acceleration.end(), 0.0);
    if (beam.kind == Kind::ModeShape) SetModeShape(beam);
    else if (beam.dynamic) SolveStatic(beam, ExternalForce(beam, 0.0, useInitialLoad));
    else SolveStatic(beam, ExternalForce(beam, 0.0, false));
    UpdateDiagnostics(beam);
}

void BeamBendingScene::UpdateDiagnostics(Beam& beam) {
    beam.maxDisplacement = 0.0;
    for (int node = 0; node <= beam.elements; ++node) beam.maxDisplacement = std::max(beam.maxDisplacement, std::abs(beam.displacement[static_cast<std::size_t>(node * 2)]));
    if (beam.support == Support::Cantilever && beam.kind == Kind::Cantilever) {
        const double ei = beam.material.youngsModulus * beam.section.Inertia();
        beam.analyticTipDeflection = beam.load * std::pow(static_cast<double>(beam.length), 3.0) / (3.0 * ei);
        const double numerical = beam.displacement.back() /* slope, replaced below */;
        (void)numerical;
    }
}

void BeamBendingScene::StepBeam(Beam& beam, double dt) {
    if (!beam.dynamic || beam.kind == Kind::ModeShape) return;
    const int fullSize = static_cast<int>(beam.displacement.size());
    const int reducedSize = static_cast<int>(beam.freeDofs.size());
    const std::vector<double> force = ExternalForce(beam, simulatedTime_, false);
    std::vector<double> applied = force;
    AddAttachmentForce(beam, applied);
    std::vector<double> uPred(static_cast<std::size_t>(fullSize), 0.0);
    std::vector<double> vPred(static_cast<std::size_t>(fullSize), 0.0);
    for (int i = 0; i < fullSize; ++i) {
        uPred[static_cast<std::size_t>(i)] = beam.displacement[static_cast<std::size_t>(i)] + dt * beam.velocity[static_cast<std::size_t>(i)] + dt * dt * (0.5 - kNewmarkBeta) * beam.acceleration[static_cast<std::size_t>(i)];
        vPred[static_cast<std::size_t>(i)] = beam.velocity[static_cast<std::size_t>(i)] + dt * (1.0 - kNewmarkGamma) * beam.acceleration[static_cast<std::size_t>(i)];
    }
    std::vector<double> matrix(static_cast<std::size_t>(reducedSize * reducedSize), 0.0);
    std::vector<double> rhs(static_cast<std::size_t>(reducedSize), 0.0);
    for (int row = 0; row < reducedSize; ++row) {
        const int globalRow = beam.freeDofs[static_cast<std::size_t>(row)];
        double residual = applied[static_cast<std::size_t>(globalRow)];
        for (int column = 0; column < fullSize; ++column) residual -= MatrixAt(beam.damping, fullSize, globalRow, column) * vPred[static_cast<std::size_t>(column)] + MatrixAt(beam.stiffness, fullSize, globalRow, column) * uPred[static_cast<std::size_t>(column)];
        rhs[static_cast<std::size_t>(row)] = residual;
        for (int column = 0; column < reducedSize; ++column) {
            const int globalColumn = beam.freeDofs[static_cast<std::size_t>(column)];
            MatrixAt(matrix, reducedSize, row, column) = MatrixAt(beam.mass, fullSize, globalRow, globalColumn) + kNewmarkGamma * dt * MatrixAt(beam.damping, fullSize, globalRow, globalColumn) + kNewmarkBeta * dt * dt * MatrixAt(beam.stiffness, fullSize, globalRow, globalColumn);
        }
    }
    std::vector<double> nextAcceleration;
    if (!SolveLinearSystem(matrix, rhs, nextAcceleration)) { solverResidual_ = std::numeric_limits<double>::infinity(); return; }
    for (int i = 0; i < fullSize; ++i) { beam.acceleration[static_cast<std::size_t>(i)] = 0.0; beam.displacement[static_cast<std::size_t>(i)] = uPred[static_cast<std::size_t>(i)]; beam.velocity[static_cast<std::size_t>(i)] = vPred[static_cast<std::size_t>(i)]; }
    for (int i = 0; i < reducedSize; ++i) {
        const int dof = beam.freeDofs[static_cast<std::size_t>(i)];
        beam.acceleration[static_cast<std::size_t>(dof)] = nextAcceleration[static_cast<std::size_t>(i)];
        beam.displacement[static_cast<std::size_t>(dof)] += kNewmarkBeta * dt * dt * nextAcceleration[static_cast<std::size_t>(i)];
        beam.velocity[static_cast<std::size_t>(dof)] += kNewmarkGamma * dt * nextAcceleration[static_cast<std::size_t>(i)];
    }
    UpdateDiagnostics(beam);
}

void BeamBendingScene::Reset() {
    simulatedTime_ = 0.0f;
    selectedStation_ = -1;
    selectedBeam_ = -1;
    dragging_ = false;
    BuildGallery();
    for (Beam& beam : beams_) ResetBeam(beam, true);
}

void BeamBendingScene::ResetStation(int station) {
    for (Beam& beam : beams_) if (beam.station == station) ResetBeam(beam, true);
}

void BeamBendingScene::Step(float dt) {
    const double clamped = std::clamp(static_cast<double>(dt), 0.0, 0.05);
    if (clamped <= 0.0) return;
    const int substeps = 2;
    const double h = clamped / static_cast<double>(substeps);
    for (int i = 0; i < substeps; ++i) {
        for (Beam& beam : beams_) StepBeam(beam, h);
        simulatedTime_ += static_cast<float>(h);
    }
    if (selectedBeam_ >= 0 && selectedBeam_ < static_cast<int>(beams_.size())) {
        const Beam& beam = beams_[static_cast<std::size_t>(selectedBeam_)];
        solverResidual_ = beam.maxDisplacement;
    }
}

double BeamBendingScene::HermiteDisplacement(const Beam& beam, double x) const {
    const double position = std::clamp(x, 0.0, static_cast<double>(beam.length));
    const double elementLength = static_cast<double>(beam.length) / static_cast<double>(beam.elements);
    const int element = std::min(beam.elements - 1, static_cast<int>(position / elementLength));
    const double s = std::clamp((position - element * elementLength) / elementLength, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const std::array<double, 4> shape = {1.0 - 3.0 * s2 + 2.0 * s3, elementLength * (s - 2.0 * s2 + s3), 3.0 * s2 - 2.0 * s3, elementLength * (-s2 + s3)};
    const int base = element * 2;
    return shape[0] * beam.displacement[static_cast<std::size_t>(base)] + shape[1] * beam.displacement[static_cast<std::size_t>(base + 1)] + shape[2] * beam.displacement[static_cast<std::size_t>(base + 2)] + shape[3] * beam.displacement[static_cast<std::size_t>(base + 3)];
}

Vector3 BeamBendingScene::BeamPoint(const Beam& beam, double x, double displacement) const { return {beam.origin.x + static_cast<float>(x), beam.origin.y + static_cast<float>(displacement * kDisplayScale), beam.origin.z}; }

void BeamBendingScene::DrawBeam(const Beam& beam) const {
    const Color reference = Fade(Color{170, 181, 201, 255}, 0.35f);
    DrawLine3D(beam.origin, {beam.origin.x + beam.length, beam.origin.y, beam.origin.z}, reference);
    constexpr int samplesPerElement = 4;
    Vector3 previous = BeamPoint(beam, 0.0, HermiteDisplacement(beam, 0.0));
    for (int sample = 1; sample <= beam.elements * samplesPerElement; ++sample) {
        const double x = static_cast<double>(sample) / static_cast<double>(beam.elements * samplesPerElement) * beam.length;
        const Vector3 current = BeamPoint(beam, x, HermiteDisplacement(beam, x));
        DrawLine3D(previous, current, beam.material.color);
        DrawLine3D(Add(previous, {0.0f, 0.0f, kBeamRadius}), Add(current, {0.0f, 0.0f, kBeamRadius}), Fade(beam.material.color, 0.45f));
        previous = current;
    }
    for (int node = 0; node <= beam.elements; ++node) {
        const double x = static_cast<double>(node) / static_cast<double>(beam.elements) * beam.length;
        DrawSphere(BeamPoint(beam, x, beam.displacement[static_cast<std::size_t>(node * 2)]), kBeamRadius * 0.62f, beam.material.color);
    }
    const Vector3 support = beam.origin;
    if (beam.support == Support::Cantilever || beam.support == Support::ClampedClamped) DrawCube({support.x - 0.08f, support.y, support.z}, 0.16f, 0.55f, 0.48f, Color{95, 105, 123, 255});
    if (beam.support == Support::ClampedClamped) DrawCube({beam.origin.x + beam.length + 0.08f, beam.origin.y, beam.origin.z}, 0.16f, 0.55f, 0.48f, Color{95, 105, 123, 255});
    if (beam.support == Support::SimplySupported) {
        DrawCylinder({beam.origin.x, beam.origin.y - 0.18f, beam.origin.z}, 0.16f, 0.16f, 0.28f, 12, Color{95, 105, 123, 255});
        DrawCylinder({beam.origin.x + beam.length, beam.origin.y - 0.18f, beam.origin.z}, 0.16f, 0.16f, 0.28f, 12, Color{95, 105, 123, 255});
    }
    if (beam.kind == Kind::DivingBoard) DrawSphere(BeamPoint(beam, beam.length, HermiteDisplacement(beam, beam.length) - 0.13), 0.14f, Color{255, 188, 92, 255});
    if (beam.kind == Kind::Bridge) {
        const double x = beam.length * (0.5 + 0.36 * std::sin(simulatedTime_ * 0.7));
        const Vector3 point = BeamPoint(beam, x, HermiteDisplacement(beam, x));
        DrawSphere(point, 0.12f, Color{255, 112, 66, 255});
        DrawLine3D(Add(point, {0.0f, 0.6f, 0.0f}), point, Color{255, 188, 92, 255});
    }
    if (beam.kind == Kind::Driven) {
        const Vector3 point = BeamPoint(beam, beam.length, HermiteDisplacement(beam, beam.length));
        DrawLine3D(Add(point, {0.0f, 0.6f, 0.0f}), point, Color{126, 235, 167, 255});
    }
}

void BeamBendingScene::Draw3D(const Camera3D& camera) const {
    (void)camera;
    DrawPlane({0.0f, 0.0f, 0.0f}, {15.0f, 10.0f}, Color{18, 26, 40, 255});
    for (int i = -7; i <= 7; ++i) {
        DrawLine3D({static_cast<float>(i), 0.012f, -4.5f}, {static_cast<float>(i), 0.012f, 4.5f}, Color{40, 52, 71, 255});
        DrawLine3D({-7.0f, 0.012f, static_cast<float>(i) * 0.64f}, {7.0f, 0.012f, static_cast<float>(i) * 0.64f}, Color{40, 52, 71, 255});
    }
    for (const Beam& beam : beams_) DrawBeam(beam);
}

void BeamBendingScene::Draw2D() const {
    const int width = GetScreenWidth();
    const int height = GetScreenHeight();
    DrawRectangle(0, 0, width, height, Color{10, 15, 27, 255});
    DrawText("PLANAR EULER-BERNOULLI BEAM VIEW", 34, 32, 22, Color{232, 238, 248, 255});
    for (int y = 0; y < kStationCount; ++y) DrawLine(30, 85 + y * 72, width - 30, 85 + y * 72, Color{40, 52, 71, 255});
    const float scaleX = std::min(width * 0.72f / 10.0f, 75.0f);
    const float offsetX = 90.0f;
    for (const Beam& beam : beams_) {
        const float rowY = 88.0f + static_cast<float>(beam.station) * 72.0f;
        Vector2 previous = {offsetX + beam.origin.x * scaleX + 460.0f, rowY - beam.origin.y * 3.0f};
        for (int sample = 1; sample <= beam.elements * 3; ++sample) {
            const double x = static_cast<double>(sample) / static_cast<double>(beam.elements * 3) * beam.length;
            const double w = HermiteDisplacement(beam, x);
            const Vector2 current = {previous.x + static_cast<float>(beam.length / (beam.elements * 3) * scaleX), rowY - static_cast<float>(w * 90.0)};
            DrawLineEx(previous, current, 3.0f, beam.material.color);
            previous = current;
        }
    }
    DrawText("display scale is visual; values remain SI", 34, height - 32, 14, Color{147, 160, 182, 255});
}

Rectangle BeamBendingScene::StationCardRectangle(int station) const { return {static_cast<float>(GetScreenWidth() - 330), 56.0f + station * 29.0f, 300.0f, 24.0f}; }
Rectangle BeamBendingScene::StationResetRectangle(int station) const { const Rectangle card = StationCardRectangle(station); return {card.x + card.width - 62.0f, card.y + 2.0f, 58.0f, card.height - 4.0f}; }

const char* BeamBendingScene::SelectedStationName() const {
    static const std::array<const char*, kStationCount> names = {{"CANTILEVER PULL", "DIVING BOARD", "MOVING LOAD BRIDGE", "MATERIAL SAMPLES", "THICKNESS SAMPLES", "PERIODIC DRIVE", "CLAMPED-CLAMPED", "MODE SHAPE"}};
    return selectedStation_ >= 0 && selectedStation_ < kStationCount ? names[static_cast<std::size_t>(selectedStation_)] : "GALLERY OVERVIEW";
}

Vector3 BeamBendingScene::FocusTarget() const {
    if (selectedStation_ < 0) return {0.0f, 2.8f, 0.0f};
    const std::array<Vector3, kStationCount> targets = {{{-4.5f, 3.9f, -1.9f}, {-1.6f, 3.9f, -1.9f}, {1.4f, 3.9f, -1.9f}, {4.1f, 3.9f, -1.9f}, {-4.5f, 1.9f, 1.55f}, {-1.6f, 2.1f, 1.55f}, {1.4f, 2.3f, 1.55f}, {4.2f, 2.35f, 1.55f}}};
    return targets[static_cast<std::size_t>(selectedStation_)];
}

bool BeamBendingScene::HandleStationClick(Vector2 mouse) {
    for (int station = 0; station < kStationCount; ++station) {
        if (!CheckCollisionPointRec(mouse, StationCardRectangle(station))) continue;
        selectedStation_ = station;
        if (CheckCollisionPointRec(mouse, StationResetRectangle(station))) ResetStation(station);
        return true;
    }
    return false;
}

Vector3 BeamBendingScene::PointOnRay(const Ray& ray, float depth) const { return Add(ray.position, Scale(ray.direction, depth)); }

int BeamBendingScene::PickBeam(const Ray& ray, float& depth, double& localX) const {
    int result = -1;
    float best = 0.18f;
    for (std::size_t index = 0; index < beams_.size(); ++index) {
        const Beam& beam = beams_[index];
        for (int sample = 0; sample <= 32; ++sample) {
            const double x = static_cast<double>(sample) / 32.0 * beam.length;
            const Vector3 point = BeamPoint(beam, x, HermiteDisplacement(beam, x));
            const Vector3 toPoint = Subtract(point, ray.position);
            const float along = Dot(toPoint, ray.direction);
            if (along <= 0.0f) continue;
            const float distance = Length(Subtract(point, PointOnRay(ray, along)));
            if (distance < best) { best = distance; result = static_cast<int>(index); depth = along; localX = x; }
        }
    }
    return result;
}

void BeamBendingScene::HandleInput(const Camera3D& camera, bool allowWorldInteraction) {
    if (IsKeyPressed(KEY_R)) Reset();
    if (IsKeyPressed(KEY_M)) { modeNumber_ = modeNumber_ % 3 + 1; for (Beam& beam : beams_) if (beam.kind == Kind::ModeShape) SetModeShape(beam); }
    if (IsKeyPressed(KEY_LEFT_BRACKET)) driveFrequency_ = std::max(0.1f, driveFrequency_ - 0.1f);
    if (IsKeyPressed(KEY_RIGHT_BRACKET)) driveFrequency_ = std::min(5.0f, driveFrequency_ + 0.1f);
    if (IsKeyPressed(KEY_MINUS)) { tipMass_ = std::max(0.1f, tipMass_ - 0.1f); for (Beam& beam : beams_) if (beam.kind == Kind::DivingBoard) { beam.tipMass = tipMass_; Assemble(beam); ResetBeam(beam, true); } }
    if (IsKeyPressed(KEY_EQUAL)) { tipMass_ = std::min(3.0f, tipMass_ + 0.1f); for (Beam& beam : beams_) if (beam.kind == Kind::DivingBoard) { beam.tipMass = tipMass_; Assemble(beam); ResetBeam(beam, true); } }
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT) && HandleStationClick(PhysicsMousePosition())) return;
    if (!allowWorldInteraction) return;
    const Ray ray = GetMouseRay(PhysicsMousePosition(), camera);
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !dragging_) {
        selectedBeam_ = PickBeam(ray, dragDepth_, dragLocalX_);
        if (selectedBeam_ >= 0) dragging_ = true;
    }
    if (dragging_ && PhysicsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        dragTargetWorldY_ = PointOnRay(ray, dragDepth_).y;
        selectedStation_ = beams_[static_cast<std::size_t>(selectedBeam_)].station;
    }
    if (dragging_ && PhysicsMouseButtonReleased(MOUSE_BUTTON_LEFT)) { dragging_ = false; selectedBeam_ = -1; }
}

void BeamBendingScene::DrawHud(bool paused) const {
    DrawRectangle(20, 20, 650, 216, Color{15, 21, 34, 232});
    DrawRectangleLines(20, 20, 650, 216, Color{76, 91, 117, 255});
    DrawText("BEAM BENDING + VIBRATION LAB", 38, 35, 22, Color{232, 238, 248, 255});
    DrawText("EULER-BERNOULLI / HERMITE FINITE ELEMENTS", 38, 68, 14, Color{120, 213, 255, 255});
    DrawText(TextFormat("beams %d   t %6.2f s   dt 1/240 s   %s", static_cast<int>(beams_.size()), simulatedTime_, paused ? "PAUSED" : "RUNNING"), 38, 98, 15, Color{196, 204, 219, 255});
    DrawText(TextFormat("selected %s   drive %4.1f Hz   tip mass %4.1f kg", SelectedStationName(), driveFrequency_, tipMass_), 38, 124, 14, Color{196, 204, 219, 255});
    DrawText("analytic supports: cantilever / simple / clamped", 38, 151, 14, Color{126, 235, 167, 255});
    DrawText("mouse drag = compliant point load; M mode; [ ] drive; -/= mass", 38, 178, 13, Color{171, 183, 201, 255});
    DrawText("R reset  P pause  N step  V 2D/3D  right-drag orbit", 38, 202, 13, Color{147, 160, 182, 255});

    const int x = GetScreenWidth() - 330;
    DrawRectangle(x - 10, 20, 320, 286, Color{15, 21, 34, 220});
    DrawRectangleLines(x - 10, 20, 320, 286, Color{76, 91, 117, 255});
    DrawText("STATIONS / CLICK TO FOCUS", x, 35, 17, Color{232, 238, 248, 255});
    static const std::array<const char*, kStationCount> names = {{"cantilever pulled/released", "diving board + tip mass", "simply supported moving load", "three material samples", "three thickness samples", "periodic driven beam", "clamped-clamped load", "selectable mode shape"}};
    for (int station = 0; station < kStationCount; ++station) {
        const Rectangle card = StationCardRectangle(station);
        const Rectangle reset = StationResetRectangle(station);
        DrawRectangleRec(card, selectedStation_ == station ? Color{28, 42, 62, 255} : Color{15, 21, 34, 255});
        DrawRectangleLinesEx(card, selectedStation_ == station ? 2.0f : 1.0f, selectedStation_ == station ? Color{120, 213, 255, 255} : Color{52, 61, 77, 255});
        DrawText(names[static_cast<std::size_t>(station)], static_cast<int>(card.x + 7), static_cast<int>(card.y + 5), 12, Color{196, 204, 219, 255});
        DrawRectangleRec(reset, Color{34, 45, 61, 255});
        DrawText("RESET", static_cast<int>(reset.x + 7), static_cast<int>(reset.y + 4), 11, Color{196, 204, 219, 255});
    }
}
