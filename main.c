// #include "ofib.h"
#include "ofib_ffi.h"

typedef struct test {
    RemoteInfo info;
} test;

int main(void) {

    struct ospf_lsa_header my_test = {
	.type_raw = 0x2001,
	.age = 0,
	.id = 2,
	.rt = 3,
	.sn = 4,
	.checksum = 5,
	.length = 6
    };

    struct ospf_lsa_header my_test2 = {
	.type_raw = 0x2001,
	.age = 1,
	.id = 2,
	.rt = 3,
	.sn = 4,
	.checksum = 5,
	.length = 6
    };

    //parse_lsa((void*) &my_test, NULL);

    struct Ofib *ofib = ofib_init(4);
    // ofib_add_lsa(ofib, (const struct ospf_lsa_header*) &my_test);
    // ofib_add_lsa(ofib, (const struct ospf_lsa_header*) &my_test2);
    // my_test2.age = 3;
    // ofib_add_lsa(ofib, (const struct ospf_lsa_header*) &my_test2);

    RemoteInfo info = {
        .node_info = {
            .src = 1,
            .depth = 2,
            .dst = 3,
        },
        .co_init = {
            .tv_sec = 10,
            .tv_nsec = 333,
        },
    };

    test *t = (test*) malloc(sizeof(test));
    t->info.node_info.src = 2;
    t->info.node_info.dst = 3;
    t->info.node_info.depth = 4;

    ofib_register_timers(ofib, info);
    ofib_register_timers(ofib, t->info);
    free(t);
    ofib_dump_timers(ofib);
    
    ofib_free(ofib);

    return 0;
}
