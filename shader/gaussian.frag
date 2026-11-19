#version 430 core

in vec4 PassColor;
in vec4 PassCov2Dinv;
in vec2 PassCenter2D;
in vec2 PassSplatUv;
flat in int PassIs2DGS;
in float PassAdjustedStdDev;
in vec3 vEyePos;

uniform float minAlpha;
uniform float falloff;
uniform bool outputSRGB;  // Apply gamma correction for sRGB framebuffer

// Portal disk clipping uniforms
uniform bool enablePortal;
uniform vec3 diskCenter;     // Disk center in camera space
uniform vec3 diskNormal;     // Disk normal in camera space
uniform float diskRadius;    // >0: behind pass (render behind disk only), <0: main pass (clip behind disk)
uniform bool diskTwoSided;   // Whether disk is two-sided

out vec4 FragColor;

void main()
{
    float z2 = dot(PassSplatUv, PassSplatUv);
    if (PassIs2DGS != 0) {
        // Preserve the existing projected 2DGS footprint.
        vec2 d = gl_FragCoord.xy - PassCenter2D;
        mat2 cov2Dinv = mat2(PassCov2Dinv.xy, PassCov2Dinv.zw);
        z2 = dot(d, cov2Dinv * d);
    }
    if (z2 > PassAdjustedStdDev * PassAdjustedStdDev) discard;
    float gaussianFalloff = exp(-0.5 * z2);
    float falloffMix = clamp(falloff, 0.0, 1.0);
    float profile = mix(1.0, gaussianFalloff, falloffMix);
    float alpha = PassColor.a;
    if (PassColor.a > 1.0) {
        float density = exp((PassColor.a * PassColor.a - 1.0) / 2.718281828459045);
        alpha = mix(1.0, 1.0 - pow(1.0 - gaussianFalloff, density), falloffMix);
    } else {
        alpha *= profile;
    }

    if (alpha < minAlpha) discard;

    // Keep the fragment path in linear premultiplied space. sRGB conversion
    // is handled uniformly by the framebuffer state so legacy and tile paths
    // share the same blend/output contract.
    vec3 color = PassColor.rgb * alpha;

    // Portal disk clipping
    if (enablePortal && diskRadius != 0.0) {
        vec3 rayDir = normalize(vEyePos);
        float denom = dot(rayDir, diskNormal);
        if (abs(denom) > 1e-8) {
            float t = dot(diskCenter, diskNormal) / denom;
            if (t > 0.0) {
                vec3 hitPoint = rayDir * t;
                vec3 diff = hitPoint - diskCenter;
                float dist2 = dot(diff, diff);
                bool insideDisk = dist2 <= diskRadius * diskRadius;
                bool behindPlane = denom < 0.0;
                if (diskRadius > 0.0) {
                    // Behind pass: only render fragments inside disk AND behind the plane
                    if (!insideDisk || !behindPlane) discard;
                } else {
                    // Main pass: discard fragments inside disk AND behind the plane
                    if (insideDisk && behindPlane) discard;
                }
            }
        }
    }

    FragColor = vec4(color, alpha);
}
