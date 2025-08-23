#pragma once
#include <cstddef>
#include <cstdint>
#include <span>

bool cobs_encode(std::span<const uint8_t> in, std::span<uint8_t> out, std::size_t& out_len) noexcept;