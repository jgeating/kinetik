Import("env")
old_flags = env["UPLOADERFLAGS"]

new_flags = [opt for opt in old_flags if opt != "--verify"]

env.Replace(
    UPLOADERFLAGS=new_flags,
    UPLOADCMD="$UPLOADER $UPLOADERFLAGS $SOURCES" 
)