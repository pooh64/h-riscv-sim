void __attribute__((naked)) _start()
{
	__asm__("la gp, __global_pointer$\n\t"
		"li sp, 1024\n\t"
		"jal ra, main\n\t"
		"ebreak\n\t");
}

int __clzsi2(int val)
{
	int i = 32;
	int j = 16;
	int temp;
	for (; j; j >>= 1) {
		if (temp = val >> j) {
			if (j == 1) {
				return (i - 2);
			} else {
				i -= j;
				val = temp;
			}
		}
	}
	return (i - val);
}

float find_pi(int prec)
{
	float sgn = (prec % 2) ? 1.0 : -1.0;
	float sum = 0;
	for (int i = prec; i > 0; --i) {
		sum += sgn / (2.0f * i - 1);
		sgn *= -1.0f;
	}
	return 4 * sum;
}

int main(int arg)
{
	return (int)(100000000L * find_pi(arg));
}
