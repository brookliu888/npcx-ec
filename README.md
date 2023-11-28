# Zephyr Applications for NPCX EC Driver Validation

This repository contains NPCX EC FW framework reference code under Zephyr. Some
of the features demonstrated in this repository are:

- Basic [Zephyr application][app_dev] skeleton
- [Zephyr workspace applications][workspace_app]
- [Zephyr modules][modules]
- [West T2 topology][west_t2]
- [Custom boards][board_porting]
- Custom [devicetree bindings][bindings]
- Out-of-tree [drivers][drivers]
- Custom [west extension][west_ext]

[app_dev]: https://docs.zephyrproject.org/latest/develop/application/index.html
[workspace_app]: https://docs.zephyrproject.org/latest/develop/application/index.html#zephyr-workspace-app
[modules]: https://docs.zephyrproject.org/latest/develop/modules.html
[west_t2]: https://docs.zephyrproject.org/latest/develop/west/workspaces.html#west-t2
[board_porting]: https://docs.zephyrproject.org/latest/guides/porting/board_porting.html
[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr
[west_ext]: https://docs.zephyrproject.org/latest/develop/west/extensions.html

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``zephyrproject``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/zephyrproject-rtos/example-application --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

### Building and running

To build the application, run the following command:

```shell
west build -b $BOARD app\`test_sutie`
```

where `$BOARD` is the npcx ec evaluation board,
`test_sutie` is the the test suite of specific driver.

You can use the `npcx9m6f_evb`, `npcx4m8f_evb`, and `npck3m7k_poc` boards found
in this repository.

Once you have built the application, run the following command to flash it:

```shell
west flash
```

#### The shortcut for Building and running

1. source env
```shell
source ../.venv/bin/activate
```
2. use `west_flash_app.sh` for build and flash
```shell
export BOARD=npck3m7k_poc
west build -p always
```
choose target board and your target app.