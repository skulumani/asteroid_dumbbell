## Vision based landing

Try to land on an asteroid using a image sensor

## Open CV setup

This will use OpenCV in Python (Anaconda)

* Export the packages in a specific `conda` environment
~~~
$ conda env export -n <env-name> > environment.yml
~~~
* Import to a new environment (on a different computer)
~~~
$ conda env create -f path/to/environment.yml
~~~

### Creating an [explicit](https://conda.io/docs/using/envs.html#share-an-environment) copy of a `conda` environment (not cross platform)

* Export a platform specific package list
~~~
$ conda list --explicit > spec-file.txt
~~~
* Import this file on the same platform (`conda` does not check this! )
~~~
$ conda create --name environment_name --file spec-file.tx
