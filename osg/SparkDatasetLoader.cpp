#include "SparkDatasetLoader.h"

#include "gf/core/gauss_ir.h"
#include "gf/io/registry.h"
#include "gf/io/reader.h"
#include "tools/miniply.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>

namespace SparkDatasetLoader
{

SparkDatasetLoadResult readSplatFile(const std::string& filename)
{
    SparkDatasetLoadResult result;
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return result;
    }

    struct Splat {
        float pos[3];
        float scale[3];
        uint8_t color[4];
        uint8_t rot[4];
    };

    Splat splat;
    while (file.read(reinterpret_cast<char*>(&splat), sizeof(Splat)))
    {
        MI_GaussianPoint g;
        g.position.set(splat.pos[0], splat.pos[1], splat.pos[2]);

        g.color.set(splat.color[0] / 255.0f, splat.color[1] / 255.0f,
                    splat.color[2] / 255.0f, splat.color[3] / 255.0f);

        // Store raw scale (already in linear space for .splat format)
        g.scale.set(splat.scale[0], splat.scale[1], splat.scale[2]);

        // Decode quaternion from uint8
        float rx = (splat.rot[0] - 128.0f) / 128.0f;
        float ry = (splat.rot[1] - 128.0f) / 128.0f;
        float rz = (splat.rot[2] - 128.0f) / 128.0f;
        float rw = (splat.rot[3] - 128.0f) / 128.0f;

        // Normalize quaternion
        float len = std::sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
        if (len > 0.0f) { rx /= len; ry /= len; rz /= len; rw /= len; }
        g.rotation.set(rx, ry, rz, rw);

        result.bounds.expandBy(g.position);
        result.points.push_back(g);
    }

    std::cout << "Loaded " << result.points.size() << " Gaussian points from .splat" << std::endl;
    return result;
}

SparkDatasetLoadResult readPlyFile(const std::string& ply_path)
{
    SparkDatasetLoadResult result;

    const float SH_C0 = 0.28209479177387814f;

    const std::vector<std::string> properties = {
        "x", "y", "z",
        "f_dc_0", "f_dc_1", "f_dc_2",
        "opacity",
        "scale_0", "scale_1", "scale_2",
        "rot_0", "rot_1", "rot_2", "rot_3",
    };

    std::vector<std::string> shProperties;
    shProperties.reserve(45);
    for (int i = 0; i < 45; ++i)
        shProperties.push_back("f_rest_" + std::to_string(i));

    miniply::PLYReader reader(ply_path.c_str());
    if (!reader.valid()) {
        std::cerr << "Failed to open " << ply_path << std::endl;
        return result;
    }

    miniply::PLYElement* elem = reader.get_element(reader.find_element("vertex"));
    if (!elem) {
        std::cerr << "No vertex element found" << std::endl;
        return result;
    }

    size_t num_gaussians = reader.num_rows();
    result.points.reserve(num_gaussians);

    std::vector<uint32_t> propIdx(properties.size());
    for (size_t i = 0; i < properties.size(); ++i)
        propIdx[i] = elem->find_property(properties[i].c_str());

    // Validate required PLY properties exist before extract_properties
    for (size_t i = 0; i < properties.size(); ++i) {
        if (propIdx[i] == miniply::kInvalidIndex) {
            std::cerr << "Missing required PLY property: " << properties[i] << std::endl;
            return result;
        }
    }

    std::vector<uint32_t> validShIdx;
    std::vector<int> validShPos;
    for (int i = 0; i < 45; ++i) {
        uint32_t idx = elem->find_property(shProperties[i].c_str());
        if (idx != miniply::kInvalidIndex) {
            validShIdx.push_back(idx);
            validShPos.push_back(i);
        }
    }
    bool fileSH = !validShIdx.empty();

    reader.load_element();
    std::vector<float> filedata(properties.size() * num_gaussians);
    reader.extract_properties(propIdx.data(), (uint32_t)properties.size(),
                              miniply::PLYPropertyType::Float, filedata.data());

    std::vector<float> shdata;
    if (fileSH) {
        int nValid = (int)validShIdx.size();
        std::vector<float> rawSH(nValid * num_gaussians, 0.0f);
        reader.extract_properties(validShIdx.data(), nValid,
                                  miniply::PLYPropertyType::Float, rawSH.data());
        shdata.resize(45 * num_gaussians, 0.0f);
        for (size_t i = 0; i < num_gaussians; ++i)
            for (int k = 0; k < nValid; ++k)
                shdata[i * 45 + validShPos[k]] = rawSH[i * nValid + k];
    }

    for (size_t i = 0; i < num_gaussians; ++i)
    {
        int offset = (int)i * (int)properties.size();
        MI_GaussianPoint g;

        g.position.set(filedata[offset + 0], filedata[offset + 1], filedata[offset + 2]);

        // DC color from SH_C0
        g.color.set(
            filedata[offset + 3] * SH_C0 + 0.5f,
            filedata[offset + 4] * SH_C0 + 0.5f,
            filedata[offset + 5] * SH_C0 + 0.5f,
            1.0f / (1.0f + std::exp(-filedata[offset + 6]))
        );

        // Store scale in log-space exponentiated
        g.scale.set(
            std::exp(filedata[offset + 7]),
            std::exp(filedata[offset + 8]),
            std::exp(filedata[offset + 9])
        );

        // Quaternion (w, x, y, z) - normalize
        float qw = filedata[offset + 10];
        float qx = filedata[offset + 11];
        float qy = filedata[offset + 12];
        float qz = filedata[offset + 13];
        float qlen = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qlen > 0.0f) { qw /= qlen; qx /= qlen; qy /= qlen; qz /= qlen; }
        g.rotation.set(qx, qy, qz, qw);

        if (fileSH) {
            int shOffset = (int)i * 45;
            for (int k = 0; k < 45; ++k)
                g.sh[k] = shdata[shOffset + k];
            g.shOrder = 3;
        }

        result.bounds.expandBy(g.position);
        result.points.push_back(g);
    }

    result.hasSH = fileSH;
    result.globalShOrder = fileSH ? 3 : 0;

    std::cout << "Loaded " << result.points.size() << " Gaussian points from .ply" << std::endl;
    return result;
}

SparkDatasetLoadResult readWithGaussForge(const std::string& filename, const std::string& ext)
{
    SparkDatasetLoadResult result;

    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return result;
    }

    size_t fileSize = (size_t)file.tellg();
    file.seekg(0);
    std::vector<uint8_t> buffer(fileSize);
    file.read(reinterpret_cast<char*>(buffer.data()), fileSize);
    file.close();

    gf::IORegistry registry;
    gf::IGaussReader* reader = registry.ReaderForExt(ext);
    if (!reader) {
        std::cerr << "No GaussForge reader for extension: " << ext << std::endl;
        return result;
    }

    gf::ReadOptions opts;
    auto readResult = reader->Read(buffer.data(), buffer.size(), opts);
    if (!readResult.ok()) {
        std::cerr << "GaussForge read error: " << readResult.error().message << std::endl;
        return result;
    }

    gf::GaussianCloudIR& ir = readResult.value();
    int N = ir.numPoints;
    int shPerPoint = (N > 0 && !ir.sh.empty()) ? (int)(ir.sh.size() / N) : 0;

    result.points.reserve(N);
    const float SH_C0 = 0.28209479177387814f;

    for (int i = 0; i < N; ++i)
    {
        MI_GaussianPoint g;

        g.position.set(
            ir.positions[i * 3 + 0],
            ir.positions[i * 3 + 1],
            ir.positions[i * 3 + 2]
        );

        g.scale.set(
            std::exp(ir.scales[i * 3 + 0]),
            std::exp(ir.scales[i * 3 + 1]),
            std::exp(ir.scales[i * 3 + 2])
        );

        // IR stores quaternion as [w, x, y, z], we store as [x, y, z, w]
        float qw = ir.rotations[i * 4 + 0];
        float qx = ir.rotations[i * 4 + 1];
        float qy = ir.rotations[i * 4 + 2];
        float qz = ir.rotations[i * 4 + 3];
        float qlen = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qlen > 0.0f) { qw /= qlen; qx /= qlen; qy /= qlen; qz /= qlen; }
        g.rotation.set(qx, qy, qz, qw);

        // Color: SH DC coefficient -> RGB [0,1] + sigmoid opacity
        g.color.set(
            0.5f + SH_C0 * ir.colors[i * 3 + 0],
            0.5f + SH_C0 * ir.colors[i * 3 + 1],
            0.5f + SH_C0 * ir.colors[i * 3 + 2],
            1.0f / (1.0f + std::exp(-ir.alphas[i]))
        );

        // SH coefficients: GaussForge stores RGB-interleaved per coefficient
        // Our shader expects channel-grouped: [R0..R14, G0..G14, B0..B14]
        std::memset(g.sh, 0, sizeof(g.sh));
        if (shPerPoint > 0)
        {
            int numCoeffs = std::min(shPerPoint / 3, 15);
            for (int c = 0; c < numCoeffs; ++c)
            {
                int srcBase = i * shPerPoint + c * 3;
                g.sh[c]            = ir.sh[srcBase + 0]; // R channel
                g.sh[c + 15]       = ir.sh[srcBase + 1]; // G channel
                g.sh[c + 30]       = ir.sh[srcBase + 2]; // B channel
            }
            if (numCoeffs >= 15)      g.shOrder = 3;
            else if (numCoeffs >= 8)  g.shOrder = 2;
            else if (numCoeffs >= 3)  g.shOrder = 1;
        }

        result.bounds.expandBy(g.position);
        result.points.push_back(g);
    }

    result.hasSH = (shPerPoint > 0);
    result.globalShOrder = 0;
    if (shPerPoint > 0) {
        int numCoeffs = std::min(shPerPoint / 3, 15);
        if (numCoeffs >= 15)      result.globalShOrder = 3;
        else if (numCoeffs >= 8)  result.globalShOrder = 2;
        else if (numCoeffs >= 3)  result.globalShOrder = 1;
    }
    std::cout << "Loaded " << result.points.size() << " Gaussian points via GaussForge (."
              << ext << ")" << std::endl;
    return result;
}

} // namespace SparkDatasetLoader
