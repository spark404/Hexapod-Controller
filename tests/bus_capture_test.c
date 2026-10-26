//
// Host test for the capture trigger and the hex dump, see Core/Src/bus_capture.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -fsanitize=address,undefined -ICore/Inc \
//      tests/bus_capture_test.c Core/Src/bus_capture.c -o /tmp/bus_capture_test && /tmp/bus_capture_test
//
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bus_capture.h"

static void test_background_failures_do_not_trigger(void)
{
    bus_capture_t c;
    bus_capture_init(&c, 5, 250);
    // one failure in a few thousand reads, as measured at rest
    for (int i = 0; i < 20000; i++) {
        assert(!bus_capture_update(&c, i % 3000 == 0));
    }
}

static void test_run_triggers_at_threshold_then_repeats(void)
{
    bus_capture_t c;
    bus_capture_init(&c, 5, 250);
    int triggers = 0;
    int first = 0;
    for (int i = 1; i <= 1000; i++) {
        if (bus_capture_update(&c, true)) {
            if (triggers == 0) {
                first = i;
            }
            triggers++;
        }
    }
    assert(first == 5);
    assert(triggers == 4);   // 5, 255, 505, 755
}

static void test_a_good_read_ends_the_run(void)
{
    bus_capture_t c;
    bus_capture_init(&c, 3, 0);
    assert(!bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, false));   // the run is broken
    assert(!bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, true));
    assert(bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, true));    // repeat 0: once per run
    assert(!bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, false));
    assert(!bus_capture_update(&c, true));
    assert(!bus_capture_update(&c, true));
    assert(bus_capture_update(&c, true));     // a new run captures again
}

static void test_hex(void)
{
    const uint8_t data[] = {0xff, 0xff, 0xfd, 0x00, 0x0a};
    char out[32];
    assert(bus_capture_hex(out, sizeof(out), data, 5) == 5);
    assert(strcmp(out, "ff ff fd 00 0a") == 0);
    assert(bus_capture_hex(out, sizeof(out), data, 0) == 0);
    assert(out[0] == '\0');

    char small[8];   // room for two bytes and the NUL: "ff ff"
    assert(bus_capture_hex(small, sizeof(small), data, 5) == 2);
    assert(strcmp(small, "ff ff") == 0);
    char one[1];
    assert(bus_capture_hex(one, sizeof(one), data, 5) == 0);
    assert(one[0] == '\0');
    assert(bus_capture_hex(one, 0, data, 5) == 0);
}

int main(void)
{
    test_background_failures_do_not_trigger();
    test_run_triggers_at_threshold_then_repeats();
    test_a_good_read_ends_the_run();
    test_hex();
    puts("bus_capture_test: ok");
    return 0;
}
