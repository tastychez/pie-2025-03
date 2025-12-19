// Render financial breakdown table
document.addEventListener("DOMContentLoaded", function () {
  const container = document.getElementById("financial-table-container");
  if (!container || typeof financialData === "undefined") return;

  const purchasedTotal = financialData.getPurchasedTotal();
  const grandTotal = financialData.getGrandTotal();

  let html = '<div class="table-wrapper"><table class="financial-table">';
  html += "<thead><tr>";
  html += "<th>Item Name</th>";
  html += '<th class="text-right">Unit Cost</th>';
  html += '<th class="text-right">Quantity</th>';
  html += '<th class="text-right">Total</th>';
  html += "</tr></thead><tbody>";

  financialData.categories.forEach((category) => {
    html += `<tr class="category-header"><td colspan="4">${category.category}</td></tr>`;
    category.data.forEach((item) => {
      html += "<tr>";
      html += `<td>${item.name}</td>`;
      html += `<td class="text-right">$${item.cost.toFixed(2)}</td>`;
      html += `<td class="text-right">${item.quantity}</td>`;
      html += `<td class="text-right">$${item.total.toFixed(2)}</td>`;
      html += "</tr>";
    });
    const categoryTotal = category.data.reduce(
      (sum, item) => sum + item.total,
      0
    );
    html += `<tr class="category-subtotal"><td colspan="3" class="text-right">${category.category} Subtotal:</td>`;
    html += `<td class="text-right">$${categoryTotal.toFixed(2)}</td></tr>`;
  });

  html += "</tbody><tfoot>";
  html += `<tr><td colspan="3" class="text-right">Total Purchased Parts:</td>`;
  html += `<td class="text-right">$${purchasedTotal.toFixed(2)}</td></tr>`;
  html += `<tr><td colspan="3" class="text-right">Total Value (including free items):</td>`;
  html += `<td class="text-right">$${grandTotal.toFixed(2)}</td></tr>`;
  html += "</tfoot></table></div>";

  container.innerHTML = html;
});

