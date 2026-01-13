# High-level orchestrator status
string phase               # IDLE | RUNNING | PAUSED | STOPPED
uint32 queue_remaining     # number of containers remaining in the queue
string container_name      # current container name
float32 container_target_g # target grams for current container
float32 weight_tolerance_g # tolerance in grams
float32 scale_weight_g     # latest scale reading (g)
