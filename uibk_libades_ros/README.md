# iis_libades_ros

Wrapper around [libades](https://github.com/IMAGINE-H2020/imagine_ades/uibk_ades)

## Usage

### Run the db :

``` 
./scripts/rosrun_adesdb_expl
```

(assumes either your database is located in $HOME or it will be created there)

### Query existing ades :

```
rosservice call /adesdb/list_ades
```
