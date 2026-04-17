#version 430 core
layout (location = 0) in vec2 inPos;

uniform samplerBuffer indexBuffer;
uniform samplerBuffer dataBuffer;
uniform samplerBuffer shBuffer;
uniform bool hasSH;

uniform mat4 view;
uniform mat4 proj;
uniform vec2 viewport_size;

out vec4 PassColor;
out vec2 PassPosition;

vec4 get_basis(mat2 sigma) {
    float a = sigma[0][0];
    float b = sigma[0][1];
    float c = sigma[1][0];
    float d = sigma[1][1];

    float tr = a + d;
    float det = a * d - b * c;

    // eigenvalues
    float s = sqrt((tr * tr) - (4 * det));
    float lambda1 = 0.5 * (tr + s);
    float lambda2 = 0.5 * (tr - s);

    // eigenvectors
    const float epsilon = 0.00001;

    vec2 e1 = vec2(1, 0);
    if (abs(c) > epsilon) {
        e1 = vec2(lambda1 - d, c);
    } else if (abs(b) > epsilon) {
        e1 = vec2(b, lambda1 - a);
    }
    e1 = normalize(e1);

    vec2 e2 = vec2(e1.y, -e1.x);

    const float max_size = 32 * 2048;
    lambda1 = min(max_size, lambda1);
    lambda2 = min(max_size, lambda2);

    // basis vectors
    vec2 b1 = sqrt(2 * lambda1) * e1;
    vec2 b2 = sqrt(2 * lambda2) * e2;

    return vec4(b1, b2);
}

// 球谐基函数系数
const float SH_C1 = 0.4886025119029199;   // 1阶
const float SH_C2[5] = float[5](          // 2阶
     1.0925484305920792,
    -1.0925484305920792,
     0.31539156525252005,
    -1.0925484305920792,
     0.5462742152960396
);
const float SH_C3[7] = float[7](          // 3阶
    -0.5900435899266435,
     2.890611442640554,
    -0.4570457994644658,
     0.3731763325901154,
    -0.4570457994644658,
     1.445305721320277,
    -0.5900435899266435
);

// 根据视角方向 dir（归一化，从高斯点指向相机）和SH系数计算RGB颜色
// dc.rgb = SH_C0 * f_dc + 0.5（CPU端已完成0阶转换），高阶部分在GPU叠加
vec3 evalSphericalHarmonics(vec3 dir, vec4 dc, int shBase)
{
    float x = dir.x, y = dir.y, z = dir.z;

    // 0阶贡献已在CPU完成，直接作为基础色
    vec3 col = dc.rgb;

    // 读取 1~3 阶 SH 系数（45个float = 12个Vec4f，从shBuffer读取）
    // 存储顺序: R(sh1..sh15), G(sh1..sh15), B(sh1..sh15)
    // 每个通道 15 个系数：1阶3个 + 2阶5个 + 3阶7个
    float sh[45];
    for (int i = 0; i < 12; ++i) {
        vec4 v = texelFetch(shBuffer, shBase + i);
        int idx = i * 4;
        if (idx + 0 < 45) sh[idx + 0] = v.x;
        if (idx + 1 < 45) sh[idx + 1] = v.y;
        if (idx + 2 < 45) sh[idx + 2] = v.z;
        if (idx + 3 < 45) sh[idx + 3] = v.w;
    }

    // 1阶 (3 coeffs per channel)
    // R: sh[0..2], G: sh[15..17], B: sh[30..32]
    col.r += SH_C1 * (-y * sh[0]  + z * sh[1]  - x * sh[2]);
    col.g += SH_C1 * (-y * sh[15] + z * sh[16] - x * sh[17]);
    col.b += SH_C1 * (-y * sh[30] + z * sh[31] - x * sh[32]);

    // 2阶 (5 coeffs per channel)
    // R: sh[3..7], G: sh[18..22], B: sh[33..37]
    float xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z;
    col.r += SH_C2[0] * xy            * sh[3]
           + SH_C2[1] * yz            * sh[4]
           + SH_C2[2] * (2*zz-xx-yy) * sh[5]
           + SH_C2[3] * xz            * sh[6]
           + SH_C2[4] * (xx-yy)       * sh[7];
    col.g += SH_C2[0] * xy            * sh[18]
           + SH_C2[1] * yz            * sh[19]
           + SH_C2[2] * (2*zz-xx-yy) * sh[20]
           + SH_C2[3] * xz            * sh[21]
           + SH_C2[4] * (xx-yy)       * sh[22];
    col.b += SH_C2[0] * xy            * sh[33]
           + SH_C2[1] * yz            * sh[34]
           + SH_C2[2] * (2*zz-xx-yy) * sh[35]
           + SH_C2[3] * xz            * sh[36]
           + SH_C2[4] * (xx-yy)       * sh[37];

    // 3阶 (7 coeffs per channel)
    // R: sh[8..14], G: sh[23..29], B: sh[38..44]
    col.r += SH_C3[0] * y*(3*xx-yy)  * sh[8]
           + SH_C3[1] * xy*z          * sh[9]
           + SH_C3[2] * y*(4*zz-xx-yy)* sh[10]
           + SH_C3[3] * z*(2*zz-3*xx-3*yy)* sh[11]
           + SH_C3[4] * x*(4*zz-xx-yy)* sh[12]
           + SH_C3[5] * z*(xx-yy)     * sh[13]
           + SH_C3[6] * x*(xx-3*yy)   * sh[14];
    col.g += SH_C3[0] * y*(3*xx-yy)  * sh[23]
           + SH_C3[1] * xy*z          * sh[24]
           + SH_C3[2] * y*(4*zz-xx-yy)* sh[25]
           + SH_C3[3] * z*(2*zz-3*xx-3*yy)* sh[26]
           + SH_C3[4] * x*(4*zz-xx-yy)* sh[27]
           + SH_C3[5] * z*(xx-yy)     * sh[28]
           + SH_C3[6] * x*(xx-3*yy)   * sh[29];
    col.b += SH_C3[0] * y*(3*xx-yy)  * sh[38]
           + SH_C3[1] * xy*z          * sh[39]
           + SH_C3[2] * y*(4*zz-xx-yy)* sh[40]
           + SH_C3[3] * z*(2*zz-3*xx-3*yy)* sh[41]
           + SH_C3[4] * x*(4*zz-xx-yy)* sh[42]
           + SH_C3[5] * z*(xx-yy)     * sh[43]
           + SH_C3[6] * x*(xx-3*yy)   * sh[44];

    // clamp到[0,1]（原始3DGS做法：加0.5偏移已在CPU完成，这里直接clamp）
    col = clamp(col, 0.0, 1.0);
    return col;
}

void main()
{
	int instanceAddress = int(texelFetch(indexBuffer,gl_InstanceID).x)*5;
	vec4 pos      = texelFetch(dataBuffer, instanceAddress+0);
	vec4 color    = texelFetch(dataBuffer, instanceAddress+1);
	vec3 sigma10  = texelFetch(dataBuffer, instanceAddress+2).xyz;
	vec3 sigma20  = texelFetch(dataBuffer, instanceAddress+3).xyz;
	vec3 sigma30  = texelFetch(dataBuffer, instanceAddress+4).xyz;

    mat3 sigma = mat3(sigma10, sigma20, sigma30);

    vec4 u = view*pos;
    u /= u.w;

    float focal =proj[0][0] * viewport_size.x * 0.5;

    mat3 jacobian = mat3(
       focal/u.z, 0.0,         -(focal * u.x) / (u.z * u.z),
        0.0,         focal/u.z, -(focal * u.y) / (u.z * u.z),
        0.0,         0.0,         0.0
    );

    // Calculate 2D covariance matrix
    mat3 t = jacobian * mat3(view);
    mat3 sigma_prime = t * sigma * transpose(t);
    mat2 sigma2 = mat2(sigma_prime);  // take upper left

    // Get basis vectors of the splatted 2D Gaussian
    vec4 bases =get_basis(sigma2);
    vec2 b1 = bases.xy;
    vec2 b2 = bases.zw;

    // 计算视角方向（世界空间，从高斯点指向相机，归一化）
    mat3 viewRot = mat3(view);
    vec3 camPos = -transpose(viewRot) * vec3(view[3].x, view[3].y, view[3].z);
    vec3 dir = normalize(camPos - pos.xyz);

    //position in screen space
    vec4 pos2d = proj * u;
    vec2 center = pos2d.xy / pos2d.w;

    gl_Position = vec4(center
        + (inPos.x * b1) / (0.5 * viewport_size)
        + (inPos.y * b2) / (0.5 * viewport_size),
        -1,1 );
    gl_Position.z = pos2d.z / pos2d.w;

    // 用球谐函数计算视角相关颜色（仅当hasSH时）
    if (hasSH) {
        int shBase = int(texelFetch(indexBuffer, gl_InstanceID).x) * 12;  // 使用排序后的索引
        vec3 shColor = evalSphericalHarmonics(dir, color, shBase);
        PassColor = vec4(shColor, color.a);
    } else {
        PassColor = color;
    }
    PassPosition = inPos;
};
