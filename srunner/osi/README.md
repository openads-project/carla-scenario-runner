# Protobuf information

## Installation

Install grpc by `pip install grpcio==1.51.1 grpcio-tools==1.44.0`

## Generating files

Generate python files from within the directory ~/scenario_runner with

```
mkdir -p srunner/osi/client/osi3
 
python3 -m grpc_tools.protoc -I ./srunner/osi/protos/osi3/ -I ./srunner/osi/protos/ --python_out ./srunner/osi/client/ --grpc_python_out ./srunner/osi/client/ ./srunner/osi/protos/ScenarioRunner.proto

:warning: generating srunner/osi/client/osi3 python-files does not work with this call:
python3 -m grpc_tools.protoc -I ./srunner/osi/protos/osi3/ -I ./srunner/osi/protos/ --python_out=./srunner/osi/client/osi3 ./srunner/osi/protos/osi3/*.proto
```
