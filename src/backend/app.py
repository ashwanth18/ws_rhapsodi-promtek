from fastapi import FastAPI, Request, BackgroundTasks
import json
import logging
from typing import List, Any, Optional
import threading
import time
import asyncio
import httpx

# Initialize FastAPI app
app = FastAPI()

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# In-memory storage for received events (replace with database in production)
events: List[Any] = []

# async def process_event(json_data: dict):
#     url = "http://localhost:8000/api/StockItemCreatedEvent"
#     async with httpx.AsyncClient() as client:
#         response = await client.post(url, json=json_data)
#         return response.json()

# POST endpoint for webhook to receive and print arbitrary JSON data
@app.post("/api/StockItemCreatedEvent")
async def receive_stock_item_created_event(request: Request, background_tasks: BackgroundTasks):
    try:
        # Read the raw JSON data from the request
        json_data = await request.json()
        
        # Print/log the received JSON data
        logger.info("Received StockItemCreatedEvent:\n%s", json.dumps(json_data, indent=2))
        
        # Store the event
        events.append(json_data)
        
        # Return acknowledgment response
        return {"message": "Event received and printed", "data": json_data}
    
    except json.JSONDecodeError:
        logger.error("Invalid JSON received")
        return {"message": "Invalid JSON format"}, 400
    except Exception as e:
        logger.error(f"Error processing webhook: %s", str(e))
        return {"message": "Error processing webhook", "error": str(e)}, 500
    finally:
        # background_tasks.add_task(process_event, json_data)
        async with httpx.AsyncClient() as client:
            response = await client.post("http://localhost:8000/api/StockItemCreatedEvent", json=json_data)
            return response.json()

# Optional: GET endpoint to verify stored events
@app.get("/api/StockItemCreatedEvent")
async def get_stored_events():
    logger.info("Fetched Stored Events:\n%s", json.dumps(events, indent=2))
    return events



# Run the app
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="localhost", port=5000)