simple C193055_HHJN
{
    parameters:
        @display("i=block/routing");
    gates:
        input in[4] @loose;
        output out @loose;
}

network C193055
{
    @display("bgb=925,577;bgi=maps/peta_bogor");

    submodules:
        // Sensor nodes
        sensor1: C193055_HHJN {
            @display("p=344,327;i=misc/sensor2");
        }
        sensor2: C193055_HHJN {
            @display("p=373,386;i=misc/sensor2");
        }
        sensor3: C193055_HHJN {
            @display("p=403,537;i=misc/sensor2");
        }
        sensor4: C193055_HHJN {
            @display("p=404,446;i=misc/sensor2");
        }
        sensor5: C193055_HHJN {
            @display("p=299,168;i=misc/sensor3,black;is=n");
        }
        sensor6: C193055_HHJN {
            @display("p=320,113;i=misc/sensor2");
        }
        sensor7: C193055_HHJN {
            @display("p=328,72;i=misc/sensor2");
        }
        sensor8: C193055_HHJN {
            @display("p=374,44;i=misc/sensor2");
        }
        sensor9: C193055_HHJN {
            @display("p=602,423;i=misc/sensor2");
        }
        sensor10: C193055_HHJN {
            @display("p=589,368;i=misc/sensor2");
        }
        sensor11: C193055_HHJN {
            @display("p=626,486;i=misc/sensor2");
        }
        sensor12: C193055_HHJN {
            @display("p=648,537;i=misc/sensor2");
        }

        // ESP32 microcontrollers
        esp32_1: C193055_HHJN {
            @display("p=432,367;i=abstract/router");
        }
        esp32_2: C193055_HHJN {
            @display("p=387,131;i=abstract/router");
        }
        esp32_3: C193055_HHJN {
            @display("p=529,357;i=abstract/router");
        }

        // Server
        server: C193055_HHJN {
            @display("p=458,250;i=device/pc");
        }

        // User node for receiving warnings
        user: C193055_HHJN {
            @display("p=600,100;i=device/laptop");
        }

    connections:
        // Connect sensors 1-4 to ESP32_1
        sensor1.out --> { delay = 100ms; } --> esp32_1.in[0];
        sensor2.out --> { delay = 100ms; } --> esp32_1.in[1];
        sensor3.out --> { delay = 100ms; } --> esp32_1.in[2];
        sensor4.out --> { delay = 100ms; } --> esp32_1.in[3];

        // Connect sensors 5-8 to ESP32_2
        sensor5.out --> { delay = 100ms; } --> esp32_2.in[0];
        sensor6.out --> { delay = 100ms; } --> esp32_2.in[1];
        sensor7.out --> { delay = 100ms; } --> esp32_2.in[2];
        sensor8.out --> { delay = 100ms; } --> esp32_2.in[3];

        // Connect sensors 9-12 to ESP32_3
        sensor9.out --> { delay = 100ms; } --> esp32_3.in[0];
        sensor10.out --> { delay = 100ms; } --> esp32_3.in[1];
        sensor11.out --> { delay = 100ms; } --> esp32_3.in[2];
        sensor12.out --> { delay = 100ms; } --> esp32_3.in[3];

        // Connect ESP32 nodes to server
        esp32_1.out --> { delay = 100ms; } --> server.in[0];
        esp32_2.out --> { delay = 100ms; } --> server.in[1];
        esp32_3.out --> { delay = 100ms; } --> server.in[2];

        // Connect server to user node for warnings
        server.out --> { delay = 100ms; } --> user.in[0];
}
