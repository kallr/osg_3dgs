#version 430 core

in vec4 PassColor;
in vec4 PassCov2Dinv;
in vec2 PassCenter2D;

out vec4 FragColor;

void main()
{
    vec2 d = gl_FragCoord.xy - PassCenter2D;
    mat2 cov2Dinv = mat2(PassCov2Dinv.xy, PassCov2Dinv.zw);
    float g = exp(-0.5 * dot(d, cov2Dinv * d));

    float alpha = PassColor.a * g;
    if (alpha < (1.0 / 255.0)) discard;

    // Premultiplied alpha output
    FragColor = vec4(PassColor.rgb * alpha, alpha);
}
