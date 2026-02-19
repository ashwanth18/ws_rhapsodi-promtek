# Processing service

This service reads MCAP bags, computes lights-out features, writes Parquet,
and posts results back to the backend.

## Run (Docker Compose)

```
docker compose up -d processing
```

## API

```
POST /process
{
  "run_db_id": 1,
  "run_folder": "/data/lightsout/20260128T184703Z_episode_1",
  "out_path": "/data/processed/run_1.parquet"
}
```


