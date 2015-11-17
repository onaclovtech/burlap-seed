# burlap-seed
Default project that should work with Ant

Currently the default usage has the burlap library with dependencies, if you want something else modify the build.xml
If you don't want to compile HelloGridWorld.java and your own file make appropriate changes (I don't know Ant that well, my file was based on an example from [ants website](https://ant.apache.org/manual/tutorial-HelloWorldWithAnt.html)). 

Run ant to clean compile and generate a jar.
    ant 

Running individual classes can happen using the command line

     java -cp ./lib/burlap.jar:./build/jar/burlap-seed.jar main.java.HelloGridWorld

     java -cp ./lib/burlap.jar:./build/jar/burlap-seed.jar main.java.GridWorldDemo

Convert to appropriate slashes,etc for Windows

