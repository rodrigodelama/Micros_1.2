void espera(int time)
{
  int i;
  for (i = 0; i < time; i++);
}

void Bin2Ascii(unsigned short number, unsigned char* chain)
{
  unsigned short partial, j, cocient, divisor;

  partial = number;
  divisor = 10000;
  *(chain) = ' ';
  for (j = 1; j < 6; j++)
  {
    cocient = partial/divisor;
    *(chain + j) = '0' + (unsigned char)cocient;
    partial = partial - (cocient*divisor);
    divisor = divisor/10;
  }
  *(chain + 6) = 0;
}

// Credit goes to GeeksForGeeks - https://www.geeksforgeeks.org/generating-random-number-range-c/
int random_num(int lower_limit, int upper_limit)
{
  int num = (rand() % (upper_limit - lower_limit + 1)) + lower_limit;
  return num;
}

