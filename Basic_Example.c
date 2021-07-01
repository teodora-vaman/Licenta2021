uint32_t counter;
void setup() {
   counter = 0;
}

void loop() {
   counter++;
}

int main() {
  kilo_init();
  kilo_start(setup, loop);
  return 0;
}