#pragma once
#include <cstdint>
#include <span>

uint32_t crc32c(std::span<const uint8_t> data) noexcept;