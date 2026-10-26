// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

//#cmake: static!

#include <unit-tests/test.h>
#include <src/core/advanced_mode.h>
#include <src/context.h>
#include <src/software-device.h>
#include <src/software-device-info.h>
#include <src/software-sensor.h>

using namespace librealsense;

namespace {
class depth_units_option : public option_base, public observable_option
{
public:
    depth_units_option() : option_base( { 0.000001f, 0.01f, 0.000001f, 0.001f } ) {}
    void set( float ) override { FAIL( "Advanced writes must not set the option again" ); }
    float query() const override { return 0.001f; }
    bool is_enabled() const override { return true; }
    const char * get_description() const override { return "Depth Units"; }
};

class advanced_mode : public ds_advanced_mode_base
{
public:
    using ds_advanced_mode_base::set;
    explicit advanced_mode( sensor_base & sensor ) { _depth_sensor = &sensor; }
    bool reject = false;
    mutable unsigned writes = 0;
    std::vector< uint8_t > send_receive( const std::vector< uint8_t > & ) const override
    {
        ++writes;
        if( reject )
            throw std::runtime_error( "Rejected advanced command" );
        return { static_cast< uint8_t >( ds::fw_cmd::SET_ADV ), 0, 0, 0 };
    }
};
}

TEST_CASE( "Advanced depth table updates the cached depth scale", "[advanced-mode]" )
{
    auto ctx = context::make( R"({"device-mask":0,"dds":{"enabled":false}})" );
    auto info = std::make_shared< software_device_info >( ctx );
    auto dev = std::make_shared< software_device >( info );
    auto & sensor = dev->add_software_sensor( "Depth" );
    auto option = std::make_shared< depth_units_option >();
    sensor.register_option( RS2_OPTION_DEPTH_UNITS, option );
    float cached_scale = 0.001f;
    unsigned notifications = 0;
    option->add_observer( [&]( float units ) { cached_scale = units; ++notifications; } );

    advanced_mode mode( sensor );
    STDepthTableControl table{};
    table.depthUnits = 2000;
    mode.set_depth_table_control( table );
    CHECK( cached_scale == Catch::Approx( 0.002f ) );
    CHECK( notifications == 1 );
    CHECK( mode.writes == 1 );

    // The lower-level overload is also used by set_all_depth for JSON/presets.
    table.depthUnits = 1000;
    mode.set( table, advanced_mode_traits< STDepthTableControl >::group );
    CHECK( cached_scale == Catch::Approx( 0.001f ) );
    CHECK( notifications == 2 );
    CHECK( mode.writes == 2 );

    mode.reject = true;
    table.depthUnits = 2000;
    CHECK_THROWS( mode.set_depth_table_control( table ) );
    CHECK( cached_scale == Catch::Approx( 0.001f ) );
    CHECK( notifications == 2 );

    mode.reject = false;
    sensor.unregister_option( RS2_OPTION_DEPTH_UNITS );
    CHECK_NOTHROW( mode.set_depth_table_control( table ) );
    sensor.register_option( RS2_OPTION_DEPTH_UNITS,
                            std::make_shared< const_value_option >( "Depth Units",
                                rsutils::lazy< float >( [] { return 0.001f; } ) ) );
    CHECK_NOTHROW( mode.set_depth_table_control( table ) );
}
