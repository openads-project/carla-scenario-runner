# Protobuf information

## Installation

Install grpc by `pip install grpcio==1.51.1 grpcio-tools==1.44.0`

## Generating files

Generate python files from within this directory with
```
 mkdir -R srunner/osi/client/osi3
 
 python -m grpc_tools.protoc  -I ./srunner/osi/protos/osi3/ -I ./srunner/osi/protos/ --python_out ./srunner/osi/client/ --grpc_python_out ./srunner/osi/client/ ./srunner/osi/protos/ScenarioRunner.proto

 python -m grpc_tools.protoc  -I ./srunner/osi/protos/osi3/ -I ./srunner/osi/protos/ --python_out ./srunner/osi/client/osi3 ./srunner/osi/protos/osi3/*.proto
```