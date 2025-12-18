/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["require-author-tag"],
  description: "File must end with one or more '> Author: … <github link>' lines.",
  information: new URL("https://github.com/UNL-Lunabotics/lunabotics-documentation/wiki/Author-Tags"),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const { lines, eol = "\n" } = params;

    let idx = lines.length - 1;
    while (idx >= 0 && !lines[idx].trim()) idx--;

    // expect an angle-bracketed GitHub URL, e.g. > Author: First Last <https://github.com/handle>
    const authorRx = /^>\s*Author:\s+[A-Z][\w.'-]*(?:\s+[A-Z][\w.'-]*)*\s+<https:\/\/github\.com\/[\w.-]+>$/;
    const footer = [];
    for (let i = idx; i >= 0 && lines[i].trim().startsWith(">"); i--) footer.unshift(lines[i]);

    const valid = footer.length > 0 && footer.every(l => authorRx.test(l));

    if (!valid) {
      const template = `> Author: First Last <https://github.com/your-handle>`;
      const fixText = (idx < 0 ? "" : eol) + template + eol;

      onError({
        lineNumber: Math.max(idx + 1, 1),
        detail: "Missing or malformed author footer.",
        context: lines[Math.max(idx, 0)] || "",
        fixInfo: (footer.length
          ? {
              editColumn: 1,
              deleteCount: -1,
              insertText: template
            }
          : {
              lineNumber: Math.max(lines.length, 1),
              editColumn: lines.length ? (lines[lines.length - 1].length + 1) : 1,
              deleteCount: 0,
              insertText: fixText
            })
      });
    }
  }
};
