#pragma once

namespace t34
{
    struct Autoflags
    {
        inline Autoflags() {};

        bool moved_from_start{};
        bool picked_up_note{};
        bool moved_to_firing_position{};
        bool fired_note{};
    };
}