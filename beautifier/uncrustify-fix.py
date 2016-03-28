import re
import sys

def main(args):
	if len(args) < 2 or len(args) > 3 or '-h' in args or '--help' in args:
		print('usage: python ' + args[0] + ' <input-path> [<output-path>]')
		return 1

	input_path = args[1]

	output_path = ''
	if len(args) > 2:
		output_path = args[2]

	lines = read_file(input_path)

	lines = namespace_ending_brackets(lines)
	lines = fix_indent(lines)

	if output_path == '':
		for line in lines:
			print(line)
	else:
		write_file(output_path, lines)

	return 0

def read_file(path):
	with open(path, 'r') as f:
		return [line.rstrip() for line in f.readlines()]

def write_file(path, lines):
	with open(path, 'w') as f:
		f.writelines([line + '\n' for line in lines])

def get_indent(line, char):
	indent = 0

	for c in line:
		if c == char:
			indent += 1
		else:
			break

	return indent

def fix_indent(lines):
	prev_indent = 0

	for i in range(len(lines)):
		if not lines[i]:
			continue

		indent = get_indent(lines[i], '\t')
		diff_indent = indent - prev_indent

		if lines[i].strip() == '{' and diff_indent > 0:
			j = i
			while True:
				if not lines[j]:
					j += 1
					continue

				if get_indent(lines[j], '\t') < indent:
					break

				lines[j] = lines[j][diff_indent:]
				j += 1

			prev_indent = get_indent(lines[i], '\t')
		else:
			prev_indent = indent

	return lines

def namespace_ending_brackets(lines):
	i = 1
	while i < len(lines):
		if re.match('^}+$', lines[i - 1]) and re.match('^}+$', lines[i]):
			lines[i - 1] += lines[i]
			del lines[i]
		else:
			i += 1

	return lines

if __name__ == '__main__':
	sys.exit(main(sys.argv))
