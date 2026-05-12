#version 430 core
layout (location = 0) in vec3 inPos;

uniform samplerBuffer indexBuffer;
uniform samplerBuffer dataBuffer;
uniform samplerBuffer shBuffer;
uniform bool hasSH;

uniform mat4 view;
uniform mat4 proj;
uniform vec2 viewport_size;

out vec4 PassColor;
out vec4 PassCov2Dinv;
out vec2 PassCenter2D;

// Compute rotation matrix from quaternion (x, y, z, w)
mat3 quatToRotation(vec4 q)
{
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x * x2, xy = x * y2, xz = x * z2;
    float yy = y * y2, yz = y * z2, zz = z * z2;
    float wx = w * x2, wy = w * y2, wz = w * z2;
    return mat3(
        1.0 - (yy + zz), xy + wz, xz - wy,
        xy - wz, 1.0 - (xx + zz), yz + wx,
        xz + wy, yz - wx, 1.0 - (xx + yy)
    );
}

// Compute 3D covariance from scale and quaternion (R * S^2 * R^T)
mat3 computeCovariance(vec3 scale, vec4 quat)
{
    mat3 R = quatToRotation(quat);
    mat3 S2 = mat3(
        scale.x * scale.x, 0.0, 0.0,
        0.0, scale.y * scale.y, 0.0,
        0.0, 0.0, scale.z * scale.z
    );
    return R * S2 * transpose(R);
}

// Invert a 2x2 matrix
mat2 inverseMat2(mat2 m)
{
    float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    return mat2(m[1][1], -m[0][1], -m[1][0], m[0][0]) / det;
}

// SH constants
const float SH_C1 = 0.4886025119029199;
const float SH_C2[5] = float[5](
    1.0925484305920792, -1.0925484305920792,
    0.31539156525252005, -1.0925484305920792, 0.5462742152960396
);
const float SH_C3[7] = float[7](
    -0.5900435899266435, 2.890611442640554, -0.4570457994644658,
    0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435
);

vec3 evalSphericalHarmonics(vec3 dir, vec3 dc, int shBase)
{
    float x = dir.x, y = dir.y, z = dir.z;
    vec3 col = dc;

    float sh[45];
    for (int i = 0; i < 12; ++i) {
        vec4 v = texelFetch(shBuffer, shBase + i);
        int idx = i * 4;
        if (idx + 0 < 45) sh[idx + 0] = v.x;
        if (idx + 1 < 45) sh[idx + 1] = v.y;
        if (idx + 2 < 45) sh[idx + 2] = v.z;
        if (idx + 3 < 45) sh[idx + 3] = v.w;
    }

    col.r += SH_C1 * (-y * sh[0]  + z * sh[1]  - x * sh[2]);
    col.g += SH_C1 * (-y * sh[15] + z * sh[16] - x * sh[17]);
    col.b += SH_C1 * (-y * sh[30] + z * sh[31] - x * sh[32]);

    float xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z;
    col.r += SH_C2[0]*xy*sh[3] + SH_C2[1]*yz*sh[4] + SH_C2[2]*(2*zz-xx-yy)*sh[5]
           + SH_C2[3]*xz*sh[6] + SH_C2[4]*(xx-yy)*sh[7];
    col.g += SH_C2[0]*xy*sh[18] + SH_C2[1]*yz*sh[19] + SH_C2[2]*(2*zz-xx-yy)*sh[20]
           + SH_C2[3]*xz*sh[21] + SH_C2[4]*(xx-yy)*sh[22];
    col.b += SH_C2[0]*xy*sh[33] + SH_C2[1]*yz*sh[34] + SH_C2[2]*(2*zz-xx-yy)*sh[35]
           + SH_C2[3]*xz*sh[36] + SH_C2[4]*(xx-yy)*sh[37];

    col.r += SH_C3[0]*y*(3*xx-yy)*sh[8] + SH_C3[1]*xy*z*sh[9]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[10] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[11]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[12] + SH_C3[5]*z*(xx-yy)*sh[13]
           + SH_C3[6]*x*(xx-3*yy)*sh[14];
    col.g += SH_C3[0]*y*(3*xx-yy)*sh[23] + SH_C3[1]*xy*z*sh[24]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[25] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[26]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[27] + SH_C3[5]*z*(xx-yy)*sh[28]
           + SH_C3[6]*x*(xx-3*yy)*sh[29];
    col.b += SH_C3[0]*y*(3*xx-yy)*sh[38] + SH_C3[1]*xy*z*sh[39]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[40] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[41]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[42] + SH_C3[5]*z*(xx-yy)*sh[43]
           + SH_C3[6]*x*(xx-3*yy)*sh[44];

    return clamp(col, 0.0, 1.0);
}

void main()
{
    // Fetch sorted instance index and per-splat data
    int sortedIdx = int(texelFetch(indexBuffer, gl_InstanceID).x);
    int dataBase = sortedIdx * 4;

    vec4 posAlpha = texelFetch(dataBuffer, dataBase + 0);
    vec4 scaleR   = texelFetch(dataBuffer, dataBase + 1);
    vec4 quatXYZG = texelFetch(dataBuffer, dataBase + 2);
    vec4 quatWB   = texelFetch(dataBuffer, dataBase + 3);

    vec3 position = posAlpha.xyz;
    float alpha   = posAlpha.w;
    vec3 scale    = scaleR.xyz;
    vec3 color    = vec3(scaleR.w, quatXYZG.w, quatWB.w);
    vec4 quat     = vec4(quatXYZG.xyz, quatWB.x);  // (x, y, z, w)

    // Transform to eye space
    vec4 eyePos = view * vec4(position, 1.0);

    // Frustum culling: discard splats behind camera
    if (eyePos.z > 0.0) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // Compute 3D covariance on GPU
    mat3 sigma = computeCovariance(scale, quat);

    // Jacobian of perspective projection
    float focal_x = proj[0][0] * viewport_size.x * 0.5;
    float focal_y = proj[1][1] * viewport_size.y * 0.5;

    mat3 J = mat3(
        focal_x / eyePos.z, 0.0, 0.0,
        0.0, focal_y / eyePos.z, 0.0,
        -(focal_x * eyePos.x) / (eyePos.z * eyePos.z),
        -(focal_y * eyePos.y) / (eyePos.z * eyePos.z),
        0.0
    );

    // Project 3D covariance to 2D: V' = J * W * sigma * W^T * J^T
    mat3 W = mat3(view);
    mat3 JW = J * W;
    mat3 sigma_prime = JW * sigma * transpose(JW);

    // Low-pass filter for anti-aliasing
    sigma_prime[0][0] += 0.3;
    sigma_prime[1][1] += 0.3;

    mat2 cov2D = mat2(sigma_prime);

    // NDC projection
    vec4 projPos = proj * eyePos;
    vec3 ndc = projPos.xyz / projPos.w;

    // Guard band culling
    if (ndc.x < -2.0 || ndc.x > 2.0 || ndc.y < -2.0 || ndc.y > 2.0) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // Compute ellipse extents from 2D covariance eigenvalues
    // https://cookierobotics.com/007/
    float a = cov2D[0][0], b = cov2D[0][1], c = cov2D[1][1];
    float apco2 = (a + c) * 0.5;
    float amco2 = (a - c) * 0.5;
    float term = sqrt(amco2 * amco2 + b * b);
    float lambda1 = apco2 + term;
    float lambda2 = apco2 - term;

    // Clamp eigenvalues
    lambda1 = max(lambda1, 0.1);
    lambda2 = max(lambda2, 0.1);

    float theta = (b == 0.0) ? ((a >= c) ? 0.0 : radians(90.0)) : atan(lambda1 - a, b);
    float k = 3.5;  // 3.5 sigma coverage
    float r1 = k * sqrt(lambda1);
    float r2 = k * sqrt(lambda2);

    vec2 majAxis = vec2(r1 * cos(theta), r1 * sin(theta));
    vec2 minAxis = vec2(r2 * cos(theta + radians(90.0)), r2 * sin(theta + radians(90.0)));

    // Offset quad vertices along ellipse axes
    vec2 offset = majAxis * inPos.x + minAxis * inPos.y;
    offset.x *= (2.0 / viewport_size.x) * projPos.w;
    offset.y *= (2.0 / viewport_size.y) * projPos.w;

    gl_Position = projPos;
    gl_Position.xy += offset;

    // Compute center in screen coordinates for fragment shader
    vec2 center = vec2(ndc.x, ndc.y);
    center.x = 0.5 * (viewport_size.x + center.x * viewport_size.x);
    center.y = 0.5 * (viewport_size.y + center.y * viewport_size.y);
    PassCenter2D = center;

    // Pass inverse 2D covariance to fragment shader
    mat2 cov2Dinv = inverseMat2(cov2D);
    PassCov2Dinv = vec4(cov2Dinv[0], cov2Dinv[1]);

    // View direction for SH evaluation
    mat3 viewRot = mat3(view);
    vec3 camPos = -transpose(viewRot) * view[3].xyz;
    vec3 dir = normalize(position - camPos);

    if (hasSH) {
        int shBase = sortedIdx * 12;
        vec3 shColor = evalSphericalHarmonics(dir, color, shBase);
        PassColor = vec4(shColor, alpha);
    } else {
        PassColor = vec4(color, alpha);
    }
}
