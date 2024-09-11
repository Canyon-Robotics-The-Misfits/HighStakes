#pragma once

namespace auto_routes
{
    enum class Route
    {
        NONE,
        POSITIVE,
        NEGATIVE,
        POSITIVE_ELIMS,
        NEGATIVE_ELIMS,
        SOLO,
    };

    void positive();
    void negative();
    void positive_elims();
    void negative_elims();
    void solo();
    void skills();
};