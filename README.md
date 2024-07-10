
Current consumption meassurements where STM8S always in run mode:

| Supply voltage [V] | Si4438 state | STM8S state | Current (Tx 10mW) [mA] | Current (Tx 50mW) [mA] | Current (Tx 100mW) [mA] |
|----:|:----:|:----:|:----:|
| 3.3 |    Tx |     Run     | 41.2 | 66.8 |    ?   |
| 3.3 | Ready |     Run     |  5.6 |  5.6 |    ?   |
| 3.3 | Sleep |     Run     |  3.5 |  3.5 |    ?   |
| 3.3 | Sleep | Active Halt |  0.2 |  0.2 |    ?   |
| 5.0 |    Tx |     Run     | 41.3 | 67.9 | 109.0* |
| 5.0 | Ready |     Run     |  5.6 |  5.7 |   5.7  |
| 5.0 | Sleep |     Run     |  3.5 |  3.5 |   3.6  |
| 5.0 | Sleep | Active Halt |  0.2 |  0.2 |   0.2  |

*Interesting, the datasheet says that Si4438 in Tx@20dBm takes ~76mA of current.
