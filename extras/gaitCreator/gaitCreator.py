# Gait Creator !

import os

step_user_input = "s"
draw_back_user_input = "db"
pause_user_input = "p"

gait_array = []

def get_code_index(lines, line):
  return lines[line].find("1")

def get_gait_code(leg_action):
  gait_scheduler_codes = open('lib/QuadrupedKinematics/src/gait-scheduler-codes.h', 'r')
  lines = gait_scheduler_codes.readlines()

  content_line = 0
  while lines[content_line] != "// --content--\n"  :
    content_line += 1
    if content_line > 50:
      print("What did you do to the content in gait-scheduler-codes.h?")
      print("Quiting")
      exit()


  step_code = int(lines[content_line + 1][lines[content_line + 1].find("1"): ].strip("\n"))
  draw_back_code = int(lines[content_line + 2][lines[content_line + 2].find("1"): ].strip("\n"))
  pause_code = int(lines[content_line + 3][lines[content_line + 3].find("1"): ].strip("\n"))

  if leg_action == step_user_input:
    return step_code
  elif leg_action == draw_back_user_input:
    return draw_back_code
  elif leg_action == pause_user_input:
    return pause_code


def print_user_codes():
  print("Select an action (step = 's', draw back = 'db', pause = 'p') for each of the 4 legs for every step.")

def main():
  gait_array.append(int(input("Enter the stride length ")))
  gait_array.append(int(input("Enter the gait amplitude ")))
  gait_array.append(int(input("Enter the duration of the gait's stride (millis) ")))
  gait_array.append(int(input("Enter the multiplication factor of the amplitude during the draw back phase\n")))
  frontmatter_end = len(gait_array) - 1

  print("Time to create the schedule! The wizard will allow you to select an action (step = 's', draw back = 'db', pause = 'p') for each of the 4 legs for every step.\n")
  print("If you forget a code, enter 'help' on any stage of the wizard.\n")
  print("If you finish, type 'done'.\n")

  step_number = 1
  schedule_complete = False
  while True:

    step = []

    print(f"In step {step_number}")

    for leg in range(1,5):
      leg_action_input = input(f"Enter the action for leg {leg}\n").lower()

      if leg_action_input == "help":
        print_user_codes()
        leg -= 1
        continue
      elif leg_action_input == "done":
        schedule_complete = True
        break
      else:
        step.append(get_gait_code(leg_action_input))

    if schedule_complete:
      break

    step_number += 1
    gait_array.append(step)
  
  gait_array_output = str(gait_array)
  gait_array_output.replace(",", "/")
  gait_array_output.replace("]", "}")
  print(gait_array_output)

if __name__ == "__main__":
  main()