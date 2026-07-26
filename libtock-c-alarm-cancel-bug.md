# Bug: `libtock_alarm_cancel` does not stop the hardware timer when cancelling the last alarm

## Summary

When `libtock_alarm_cancel` is called on the only remaining alarm in the list,
the hardware timer is not stopped. The timer fires anyway and delivers a stale
upcall to `alarm_upcall`, which immediately hits an assertion failure because
the alarm list is empty.

## Affected file

`libtock/services/alarm.c` — `libtock_alarm_cancel`

## Root cause

When an alarm that is the head of the list is cancelled and has no successor,
`root` is set to `NULL`. At that point the code does nothing further:

```c
void libtock_alarm_cancel(libtock_alarm_ticks_t* alarm) {
  ...
  if (root == alarm) {
    root = alarm->next;          // becomes NULL
    if (root != NULL) {
      libtock_alarm_command_set_absolute(root->reference, root->dt);
      // only branch taken; the else case is missing
    }
    // hardware timer is still armed and will fire
  }
  ...
}