# brew install protobuf  (this command will install protoc, allowing you to compile the protobuf files)
protobuf: pacmanState.proto lightState.proto pacmanCommand.proto test.proto pacCommand.proto ack.proto
	protoc -I=./ --python_out=./ ./pacmanState.proto
	protoc -I=./ --python_out=./ ./lightState.proto
	protoc -I=./ --python_out=./ ./pacmanCommand.proto
	protoc -I=./ --python_out=./ ./test.proto
	protoc -I=./ --python_out=./ ./pacCommand.proto
	protoc -I=./ --python_out=./ ./ack.proto

