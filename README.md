# burlap-seed
Default project that should work with Ant


Currently the default usage has the burlap library with dependencies, if you want something else modify the build.xml
If you don't want to compile HelloGridWorld.java and your own file make appropriate changes (I don't know Ant that well, my file was based on an example from ants website). 

Basically if you just run ant alone you'll compile your stuff, if you run ant run it'll run your resulting .jar or you can call out the specific steps, if you'd like.

Each can be ran independently or together.

    ant clean build run
