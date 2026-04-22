#pragma once
#include "../include/glm/glm.hpp"

class texture {
public:
    virtual ~texture() = default;

    virtual glm::vec3 value(double u, double v, const glm::vec3& p) const = 0;
};

class solid_color : public texture {
public:
    solid_color(const glm::vec3& albedo) : albedo(albedo) {}

    solid_color(double red, double green, double blue) : solid_color(glm::vec3(red, green, blue)) {}

    glm::vec3 value(double u, double v, const glm::vec3& p) const override {
        return albedo;
    }

private:
    glm::vec3 albedo;
};

class checker_texture : public texture {
public:
    checker_texture(double scale, texture* even, texture* odd)
        : inv_scale(1.0 / scale), even(even), odd(odd) {
    }

    checker_texture(double scale, const glm::vec3& c1, const glm::vec3& c2)
        : checker_texture(scale, new solid_color(c1), new solid_color(c2)) {
    }

    glm::vec3 value(double u, double v, const glm::vec3& p) const override {
        auto xInteger = int(std::floor(inv_scale * p.x));
        auto yInteger = int(std::floor(inv_scale * p.y));
        auto zInteger = int(std::floor(inv_scale * p.z));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? even->value(u, v, p) : odd->value(u, v, p);
    }

private:
    double inv_scale;
    texture* even;
    texture* odd;
};