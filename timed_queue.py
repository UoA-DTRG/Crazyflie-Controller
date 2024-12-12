import threading
import time
import queue
from queue import Empty


class TimedQueue:
    def __init__(self, max_wait_time: int = 10):
        """
        Initialize a timed queue with reliable element tracking.

        :param max_wait_time: Maximum time (in seconds) a task can wait before being discarded
        """
        self._queue = queue.Queue()
        self._max_wait_time = max_wait_time
        self._stop_event = threading.Event()

        # Thread-safe element tracking
        self._lock = threading.Lock()
        self._element_count = 0

        # Timeout monitoring thread
        self._timeout_thread = threading.Thread(target=self._timeout_monitor, daemon=True)
        self._timeout_thread.start()

    def put(self, item: any):
        """
        Put an item in the queue with timestamp tracking.

        :param item: Task to be added to the queue
        """
        current_time = time.time()

        # Use lock to ensure thread-safe counting
        with self._lock:
            self._queue.put((item, current_time))
            self._element_count += 1

    def get(self, block: bool = True, timeout: float = None) -> Any:
        """
        Retrieve and remove the next available task from the queue.

        :param block: Whether to block if queue is empty
        :param timeout: Maximum time to wait for an item
        :return: The next task from the queue
        :raises queue.Empty: If no item is available
        """
        try:
            # Retrieve the item
            item, _ = self._queue.get(block=block, timeout=timeout)

            # Decrement the element count
            with self._lock:
                self._element_count -= 1

            return item
        except queue.Empty:
            raise

    def qsize(self) -> int:
        """
        Get the current number of elements in the queue.

        :return: Number of elements in the queue
        """
        with self._lock:
            return self._element_count

    def _timeout_monitor(self):
        """
        Background thread to monitor and remove timed-out tasks.
        """
        while not self._stop_event.is_set():
            current_time = time.time()

            # Temporary queue and count for non-timed-out tasks
            temp_queue = queue.Queue()
            temp_count = 0

            # Process the existing queue
            while not self._queue.empty():
                try:
                    task, task_timestamp = self._queue.get(block=False)

                    # Keep tasks that haven't exceeded the timeout
                    if current_time - task_timestamp <= self._max_wait_time:
                        temp_queue.put((task, task_timestamp))
                        temp_count += 1
                except queue.Empty:
                    break

            # Update queue and count atomically
            with self._lock:
                self._queue = temp_queue
                self._element_count = temp_count

            # Sleep to prevent continuous CPU usage
            time.sleep(1)

    def close(self):
        """
        Stop the timeout monitoring thread.
        """
        self._stop_event.set()
        self._timeout_thread.join()