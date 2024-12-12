import threading
import time
import queue
from queue import Empty


class TimedQueue:
    def __init__(self, max_wait_time: int = 0.02):
        """
        Thread-safe queue with timeout functionality.

        :param max_wait_time: Maximum time (in seconds) a task can wait before being discarded
        """
        self._queue = queue.Queue()
        self._max_wait_time = max_wait_time
        self._stop_event = threading.Event()
        self._timeout_thread = threading.Thread(target=self._timeout_monitor, daemon=True)
        self._timeout_thread.start()

    def put(self, item):
        """
        Thread-safe method to add an item to the queue.

        :param item: Task to be added to the queue
        """
        self._queue.put((item, time.time()))

    def get(self, block=True, timeout=None):
        """
        Thread-safe method to retrieve an item from the queue.

        :param block: Whether to block if queue is empty
        :param timeout: Maximum time to wait for an item
        :return: The next task from the queue
        :raises Empty: If no item is available and not blocking
        """
        try:
            return self._queue.get(block=block, timeout=timeout)[0]
        except Empty:
            raise

    def _timeout_monitor(self):
        """
        Background thread to monitor and remove timed-out tasks.
        """
        while not self._stop_event.is_set():
            current_time = time.time()

            # Temporary list to hold tasks that should be kept
            temp_queue = queue.Queue()

            # Process the existing queue
            while not self._queue.empty():
                try:
                    task, task_timestamp = self._queue.get(block=False)

                    # Keep tasks that haven't exceeded the timeout
                    if current_time - task_timestamp <= self._max_wait_time:
                        temp_queue.put((task, task_timestamp))
                except queue.Empty:
                    break

            # Replace the original queue with filtered tasks
            self._queue = temp_queue

            # Sleep to prevent continuous CPU usage
            time.sleep(1)

    def close(self):
        """
        Stop the timeout monitoring thread.
        """
        self._stop_event.set()
        self._timeout_thread.join()