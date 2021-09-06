# Use PID_Component below to declare tests and run_PID_Test to define how to run them
# The full documentation is available here https://pid.lirmm.net/pid-framework/assets/apidoc/html/pages/Package_API.html
#
# Common usages, but tweek them to your needs
#
# PID_Component(
#   my-test
#   DESCRIPTION ...
#   DEPEND <list of dependencies>
#   C_STANDARD 11
#   CXX_STANDARD 17
#   WARNING_LEVEL MORE
# )
# 
# run_PID_Test(
#   NAME checking-feature-x
#   COMPONENT my-test
#   ARGUMENTS "foox" "barx"
# )
# 
# run_PID_Test(
#   NAME checking-feature-y
#   COMPONENT my-test
#   ARGUMENTS "fooy" "bary"
# )
